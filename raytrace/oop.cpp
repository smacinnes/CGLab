#include "OpenGP/Image/Image.h"
#include "bmpwrite.h"

using namespace OpenGP;

using Colour = Vec3; // RGB Value
Colour red()    { return Colour(1.0f, 0.0f, 0.0f); }
Colour green()  { return Colour(0.0f, 1.0f, 0.0f); }
Colour blue()   { return Colour(0.0f, 0.0f, 1.0f); }
Colour white()  { return Colour(1.0f, 1.0f, 1.0f); }
Colour black()  { return Colour(0.0f, 0.0f, 0.0f); }

// bind a single pixel value in range [0.0f,1.0f]
float bindPixel(float value){
    if (value <= 0)    return 0.0f;
    if (value >= 1.0f) return 1.0f;
    return value;
}
// bind a all pixel values in range [0.0f,1.0f]
Vec3 bindPixelValues(const Vec3& v){
    return Vec3(bindPixel(v(0)),bindPixel(v(1)),bindPixel(v(2)));
}
// multiply two vectors elementwise
Vec3 mult(const Vec3& v1,const Vec3& v2){
    return Vec3(v1(0)*v2(0),v1(1)*v2(1),v1(2)*v2(2));
}
// print a vector for debugging purposes
void printVec(const Vec3& v){
    printf("%.3f  %.3f  %.3f",double(v(0)),double(v(1)),double(v(2)));
}
void printlnVec(const Vec3& v){
    printf("%.3f  %.3f  %.3f\n",double(v(0)),double(v(1)),double(v(2)));
}
// if two floats are close enough
bool equal(float a, float b, float e){
    return fabs(a-b) <= e;
}



/*
 *  The point (0,0,0) will be an absolute position in space and is not defined
 *  relative to the camera, image plane or anything else in the scene. The
 *  distance from the camera to the image plane is defined as 1.0 (but this
 *  can be changed under ImagePlane) and all other dimensions will be scaled
 *  off of that distance.
 */




// Class defining the position and orientation of the camera
// TODO: add easier adjustment of tilting function (ex auto calculate up
// vector from the tilt in radians)
class Camera {
public:
    // set these three manually
    Vec3 position =  Vec3(0,0,5);   // location of body of camera
    Vec3 lookingAt = Vec3(0,0,4);   // where the camera is facing
    Vec3 up = Vec3(0,1,0);          // tilt of camera: (0,1,0) is no tilt
    // these will be set during setup() call
    Vec3 viewingDir = Vec3(0,0,0);
    Vec3 rightAxis = Vec3(0,0,0);
    Vec3 upAxis = Vec3(0,0,0);

    void setup(){
        viewingDir = (lookingAt - position).normalized();
        rightAxis = (up.cross(viewingDir)).normalized();
        upAxis = viewingDir.cross(rightAxis); // will be unit vec
    }
};

// Class defining all properties of the image plane
class ImagePlane {
private:
    // set these manually
    int wResolution = 640;      // default
    int hResolution = 480;      // default
    float viewingAngle = 90.0f; // recommended - most natural for humans
    float distToCam = 1.0f;     // all other dimensions based off this
    // these are calculated automatically
    Vec3 center = Vec3(0,0,0);
    float hwRatio = float(hResolution)/wResolution;
    float halfWidth = 0.0f;
public:
    Image<Colour> image = Image<Colour>(hResolution, wResolution);
    Vec3 llc = Vec3(0,0,0);   // lower left corner
    Vec3 pixRi = Vec3(0,0,0); // move one pixel to the right
    Vec3 pixUp = Vec3(0,0,0); // move one pixel up

    void setup(const Camera& c){
        center = c.position+c.viewingDir*distToCam;
        halfWidth = distToCam*tanf(viewingAngle/2.0f);
        llc = center - c.rightAxis*halfWidth - c.upAxis*halfWidth*hwRatio;
        pixRi = 2*halfWidth/(wResolution-1)*c.rightAxis;
        pixUp = 2*halfWidth*hwRatio/(hResolution-1)*c.upAxis;
    }
};

class Ray {
public:
    Vec3 orig = Vec3(0,0,0);
    Vec3 dir = Vec3(0,0,0);

    // intersect image plane at specified pixel
    void constructPrimary(const ImagePlane& im,int row,int col,const Camera& cam){
        orig = im.llc + im.pixRi*float(col-1) + im.pixUp*float(row-1);
        // pixel direction as unit vector
        dir = (orig - cam.position).normalized();
    }
};

// All objects in the scene will have these material properties
class Obj {
public:
    Colour ka = Colour(0.0f, 0.0f, 0.0f);
    Colour kd = Colour(0.0f, 0.0f, 0.0f);
    Colour ks = Colour(0.0f, 0.0f, 0.0f);
    float alpha = 0.0f;

    // can these be const references? try both
    Obj(Vec3 a,Vec3 d,Vec3 s,float al) :
        ka(a),
        kd(d),
        ks(s),
        alpha(al) {}

    // if there is a valid intersection
    virtual bool intersects(const Ray&) = 0;
    // if there is a valid intersection
    virtual float intersects2(const Ray&) = 0;
    // what that intersection is
    virtual Vec3 intersection(const Ray&) = 0;
    // normal at a point
    virtual Vec3 normal(const Vec3&) const = 0;
    // virtual destructor
    virtual ~Obj() {}

};

class Plane : public Obj{
public:
    Vec3 point = Vec3(0,0,0);
    Vec3 norm = Vec3(0,1,0);
    float temp = 0.0f;

    Plane(Vec3 p,Vec3 n,Vec3 a,Vec3 d,Vec3 s,float al) :
        Obj(a,d,s,al),
        point(p),
        norm(n){}

    // returns true iff the ray has a valid intersection with the given plane
    // "valid" meaning a single positive solution
    bool intersects(const Ray& r){
        temp = norm.dot(r.dir);                     // hold dot prod op
        if(equal(temp,0.0f,0.001f)) return false;   // if dot==0
        temp = (point-r.orig).dot(norm)/temp;       // hold scalar solution
        return temp > 0;                            // ensure (+) solution
    }

    // returns the distance the ray travelled before a valid intersection
    // returns -1 if no intersection or invalid intersection
    // "valid" meaning a single positive solution
    float intersects2(const Ray& r){
        temp = norm.dot(r.dir);                     // hold dot prod op
        if(equal(temp,0.0f,0.001f)) return -1.0f;   // if dot==0
        temp = (point-r.orig).dot(norm)/temp;       // t in x = o+td
        if (temp < 0) return -1.0f;
        return temp;
    }


    // assumes intersects() has just been evaluated to true
    // therefore temp holds the positive distance to the line
    Vec3 intersection(const Ray& r){
        temp = norm.dot(r.dir);                     // hold dot prod op   // if dot==0
        temp = (point-r.orig).dot(norm)/temp;       // hold scalar solution

        return r.orig+temp*r.dir;
    }
    // return normal
    Vec3 normal(const Vec3&) const {
        return norm;
    }
};

class Sphere : public Obj {
public:
    Vec3 center = Vec3(0,0,0);
    float radius = 0.0f;
    Vec3 co = Vec3(0,0,0);      //center of object to origin of ray
    float disc = 0.0f;
    float t1 = 0.0f;
    float t2 = 0.0f;
    float b = 0.0f;

    Sphere(Vec3 c,float r,Vec3 a,Vec3 d,Vec3 s,float al) :
        Obj(a,d,s,al),
        center(c),
        radius(r){}

    // returns true iff the ray has a valid intersection with the given sphere
    // "valid" meaning that at least one solution is positive
    // two pos solutions:   ray starts before sphere    (desired)
    // one pos solution:    ray starts inside sphere    (cross section - optional)
    // two neg solution:    ray starts after sphere     (ignore)
    // two comp solutions:  ray does not intersect      (ignore)
    // this function stores the discriminant value and any solution values
    // it calculates to avoid unnecessary recalculations
    bool intersects(const Ray& r){
        co = r.orig-center;
        disc = powf(r.dir.dot(co),2)-powf(co.norm(),2)+powf(radius,2);
        if (disc < 0.0f) return false;
        b = -(r.dir.dot(co));
        t1 = b+sqrt(disc);
        t2 = b-sqrt(disc);
        return t1 > 0;                  // change to t2 to avoid cross sections
    }


    // returns true iff the ray has a valid intersection with the given sphere
    // "valid" meaning that at least one solution is positive
    // two pos solutions:   ray starts before sphere    (desired)
    // one pos solution:    ray starts inside sphere    (cross section - optional)
    // two neg solution:    ray starts after sphere     (ignore)
    // two comp solutions:  ray does not intersect      (ignore)
    // this function stores the discriminant value and any solution values
    // it calculates to avoid unnecessary recalculations
    float intersects2(const Ray& r){
        co = r.orig-center;
        disc = powf(r.dir.dot(co),2)-powf(co.norm(),2)+powf(radius,2);
        if (disc < 0.0f) return -1.0f;
        b = -(r.dir.dot(co));
        t1 = b+sqrt(disc);
        t2 = b-sqrt(disc);
        if (t1 < 0) return -1.0f;
        //if (t2 < 0) return t1;          // remove this line for no cross sections
        return t2;
    }



    // assumes intersects() has already been evaluated to true
    // therefore disc, t1 and t2 have been calculated
    // add error message printouts for invalid disc and t?
    Vec3 intersection(const Ray& r){
        co = r.orig-center;
        disc = powf(r.dir.dot(co),2)-powf(co.norm(),2)+powf(radius,2);
        b = -(r.dir.dot(co));
        t1 = b+sqrt(disc);
        t2 = b-sqrt(disc);

        if (t2>0) return r.orig+t2*r.dir;
        return r.orig+t1*r.dir;
    }
    // return normal
    Vec3 normal(const Vec3& p) const {
        return (p-center).normalized();
    }
};

class LightSource {
public: // properties
    Vec3 position = Vec3(0,0,0);
    Colour ambient = Colour(0.0f, 0.0f, 0.0f);
    Colour diffuse = Colour(0.0f, 0.0f, 0.0f);
    Colour specular = Colour(0.0f, 0.0f, 0.0f);
    // for calculations
    Vec3 normal = Vec3(0,0,0);
    Vec3 lightAngle = Vec3(0,0,0);
    Vec3 R = Vec3(0,0,0);
    Colour Ia = Colour(0.0f, 0.0f, 0.0f);
    Colour Id = Colour(0.0f, 0.0f, 0.0f);
    Colour Is = Colour(0.0f, 0.0f, 0.0f);

    // ensure all light levels fall in correct range
    void bindValues(){
        Ia = bindPixelValues(Ia);
        // if diffuse <= 0, specular must be 0
        for(int i=0;i<3;i++) if(Id(i)<0.0f) Is(i)=0.0f;
        Id = bindPixelValues(Id);
        Is = bindPixelValues(Is);
    }

    Colour illuminate(const Obj* o,const Ray& r,const Vec3& intersection){
        // ambient light calculation
        Ia = mult(o->ka,ambient);

        // diffused light calculation
        normal = o->normal(intersection);
        lightAngle = (intersection - position).normalized();
        Id = mult(lightAngle.dot(normal)*o->kd,diffuse);

        // specular light calculation
        R = 2*lightAngle.dot(normal)*normal-lightAngle;
        Is = mult(o->ks*pow(R.dot(-r.dir),o->alpha),specular);

        bindValues();
        return (Ia + Id + Is);
    }
};

// remember destructors or will have memory leak
class Objs {
public:
    std::vector<Obj*> objects;
    float minDist = std::numeric_limits<float>::max();
    float temp = 0.0f;
    Vec3 t = Vec3(0,0,0);
    int indexClosest = -1;

    Objs(std::vector<Plane*> p,std::vector<Sphere*> s){
        // makes copies, inefficient?
        for(auto it=p.begin();it!=p.end();++it){
            objects.push_back(*it);
        }
        for(auto it=s.begin();it!=s.end();++it){
            objects.push_back(*it);
        }
    }

    // do this with intersects or intersects2?
    Obj* findFirstObject(const Ray& r){
        for(int i=0;i<objects.size();++i){
            if (objects[i]->intersects(r)){
                t = objects[i]->intersection(r);
                temp = (t-r.orig).norm();
                if (temp < minDist){
                    minDist = temp;
                    indexClosest = i;
                }
            }
        }
        /*
        for(int i=0;i<objects.size();++i){
            temp = objects[i]->intersects2(r);
            if (0.0f <= temp && temp < minDist){
                minDist = temp;
                indexClosest = i;
            }
        }
        */
        if (indexClosest == -1) return nullptr;
        return objects.at(indexClosest);
    }
};

// sets up the camera and image plane
// when moving objects in the scene, the x-direction is reversed from what
// would be intuitive, except when moving the light sources, in which case
// the x-axis is intuitive but the others are not. Correct that behavior here
// so the user can define all objects intuitively
void setup(Camera& cam,ImagePlane& im,std::vector<Sphere*> &spheres,LightSource& L){
    cam.position    = mult(cam.position, Vec3(-1,1,1));
    cam.lookingAt   = mult(cam.lookingAt,Vec3(-1,1,1));

    for(auto it=spheres.begin();it!=spheres.end();++it){
        (*it)->center = mult((*it)->center, Vec3(-1,1,1));
    }

    L.position = mult(L.position,Vec3(1,-1,-1));

    cam.setup();
    im.setup(cam);
}

int main(int, char**){
    // (+x: right +y: up +z: out of screen)

    // DEFINE CAMERA
    // {position,lookingAt}
    Camera cam = {Vec3(0,0,4),Vec3(0,0,3)};

    // DEFINE IMAGE PLANE
    // default is 640x480 w/ 90 deg view angle
    ImagePlane im;

    // DEFINE FLOOR
    // {point,normal,ambient,diffuse,specular,alpha}
    //Plane floor(Vec3(0,-2,0),Vec3(0,1,0),Colour(.2f,.8f,.2f),Colour(.2f,.8f,.2f),Colour(.2f,.8f,.2f),1);
    std::vector<Plane*> planes;
    //planes.push_back(&floor);

    // DEFINE OBJECTS IN SCENE
    // {position,radius,ambient,diffuse,specular,alpha}
    Sphere s1 = {Vec3(-2,0,0),1,Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),5};
    Sphere s2 = {Vec3(0,0,0),1,Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),5};
    std::vector<Sphere*> spheres;
    spheres.push_back(&s1);
    spheres.push_back(&s2);

    // DEFINE LIGHTING SOURCES - eventually do multiple
    // {position,ambient,diffuse,specular}
    LightSource L = {Vec3(5,5,2),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f)};

    setup(cam,im,spheres,L);

    Objs objects = {planes,spheres};

    // INITIALIZE LOOP VARIABLES
    Ray pixel;
    Obj* closest = nullptr;

    // add test for image plane intersecting any objects?


    for (int row = 0; row < im.image.rows(); ++row) {
        for (int col = 0; col < im.image.cols(); ++col) {

            pixel.constructPrimary(im,row,col,cam);

            closest = objects.findFirstObject(pixel);

            if (closest!=nullptr) {
                im.image(row,col) = L.illuminate(closest,pixel,closest->intersection(pixel));

            } else {
                im.image(row,col) = black();
            }


        }
    }

    bmpwrite("../../out.bmp", im.image);
    imshow(im.image);

    return EXIT_SUCCESS;
}

