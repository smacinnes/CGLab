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
// multiply two vectors elementwise (useful for colours)
Vec3 mult(const Vec3& v1,const Vec3& v2){
    return Vec3(v1(0)*v2(0),v1(1)*v2(1),v1(2)*v2(2));
}
// print a vector
void printVec(const Vec3& v){
    printf("%.3f  %.3f  %.3f\n",double(v(0)),double(v(1)),double(v(2)));
}
// if two floats are close enough
bool equal(float a, float b, float e){
    return std::abs(a-b) <= e;
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
        printf("Camera:");
        printVec(position);
    }
};

// Class defining all properties of the image plane
// For simplicity in contructing the environment, the dist from the image plane
// to the camera is defined as 1.0
class ImagePlane {
private:
    // set these manually
    int wResolution = 640;      // default is 640
    int hResolution = 480;      // default is 480
    float viewingAngle = 90.0f; // changing this causes strange behavior
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

        printf("\nCorners of Image Plane are:\n");
        printf("LLC: ");
        printVec(llc);
        printf("URC: ");
        printVec(center+c.rightAxis*halfWidth+c.upAxis*halfWidth*hwRatio);
        printf("Dist to Cam: %.1f\n",double(distToCam));
        printf("View Angle: %.1f\n",double(viewingAngle));
    }
};

// the intersect() and related methods are stored in the object class definition
// for purposes of overloading
class Ray {
public:
    Vec3 orig = Vec3(0,0,0);
    Vec3 dir = Vec3(0,0,0);
public:

    // create a Ray between these two points
    // offset is distance along ray to move the start position
    // to avoid self intersection
    // return the distance between the points
    float createBetween(const Vec3& from, const Vec3& to, float offset){
        dir = (to-from).normalized();
        orig = from+offset*dir;
        return (to-orig).norm();
    }
    // intersect image plane at specified pixel
    void constructPrimary(const ImagePlane& im,int row,int col,const Camera& cam){
        // this way makes the ray start at the image plane
        // good for cross sections, bad for realism
        /* orig = im.llc + im.pixRi*float(col-1) + im.pixUp*float(row-1);
         * dir = (orig - cam.position).normalized(); */
        // this way makes the ray start at the camera
        // good for realism, cant do cross sections
        orig = cam.position;
        dir = (im.llc+im.pixRi*float(col-1)+im.pixUp*float(row-1)
               -cam.position).normalized();
    }
    // ray location with given parameter
    Vec3 at(float t) const {
        return orig+t*dir;
    }
};

// All objects in the scene will derive from this class and
// have these material properties and methods for simple design
class Obj {
public:
    Colour ka = Colour(0.0f, 0.0f, 0.0f);
    Colour kd = Colour(0.0f, 0.0f, 0.0f);
    Colour ks = Colour(0.0f, 0.0f, 0.0f);
    float alpha = 0.0f;

    // can these be const references?
    Obj(Vec3 a,Vec3 d,Vec3 s,float al) :
        ka(a),
        kd(d),
        ks(s),
        alpha(al) {}

    // returns the dist to the valid intersection if there is one, -1 otherwise
    // ray is o+td, this returns t or -1
    virtual float intersects(const Ray&) = 0;

    // returns the normal at the specified point
    virtual Vec3 normal(const Vec3&) const = 0;

    // virtual destructor - compiler complains without it
    virtual ~Obj() {}
};

// defines an infinite plane (ex the floor)
// use triangles (yet to be implemented) for walls
class Plane : public Obj{
public:
    Vec3 point = Vec3(0,0,0);
    Vec3 norm = Vec3(0,1,0);
private:
    float temp = 0.0f;
public:
    Plane(Vec3 p,Vec3 n,Vec3 a,Vec3 d,Vec3 s,float al) :
        Obj(a,d,s,al),
        point(p),
        norm(n.normalized()){
        printf("Plane: ");
        printVec(point);
        printVec(norm);
    }

    /* returns the distance along the ray from the plane to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning a single positive solution
     */
    float intersects(const Ray& r){
        temp = norm.dot(r.dir);                     // hold dot prod op
        if(equal(temp,0.0f,0.0001f)) return -1.0f;  // if dot~=0
        temp = (point-r.orig).dot(norm)/temp;       // hold scalar solution
        if (temp > 0) return temp;                  // ensure (+) solution
        return -1.0f;
    }

    // return normal (parameter there for inheritance)
    Vec3 normal(const Vec3&) const {
        return norm;
    }
};

// defines a sphere via a point and a radius
class Sphere : public Obj {
public:
    Vec3 center = Vec3(0,0,0);
    float radius = 0.0f;
private:
    Vec3 co = Vec3(0,0,0);      // center of sphere to origin of ray
    float disc = 0.0f;
    float t1 = 0.0f;
    float t2 = 0.0f;
    float b = 0.0f;
public:
    Sphere(Vec3 c,float r,Vec3 a,Vec3 d,Vec3 s,float al) :
        Obj(a,d,s,al),
        center(c),
        radius(r){
        printf("Sphere: ");
        printVec(center);
        printf("%.2f",double(radius));
    }

    /* returns the distance along the ray from the sphere to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning that at least one solution is positive
     * two pos:    ray starts before sphere    (return smallest solution)
     * one pos:    ray starts inside sphere    (cross section - optional)
     * two neg:    ray starts after sphere     (ignore)
     * no real:    ray does not intersect      (ignore)
     */
    float intersects(const Ray& r){
        co = r.orig-center;
        disc = powf(r.dir.dot(co),2)-powf(co.norm(),2)+powf(radius,2);
        if (disc < 0.0f) return -1.0f; // complex solutions, no intersection
        b = -(r.dir.dot(co));
        t1 = b+sqrt(disc);
        t2 = b-sqrt(disc);
        if (t1 < 0) return -1.0f;       // ray starts after sphere
        // to avoid cross sections, change the previous line to read "t2" and
        // comment out the following line
        if (t2 < 0) return t1;
        return t2;
    }

    // return normal
    Vec3 normal(const Vec3& p) const {
        return (p-center).normalized();
    }
};

// remember destructors or will have memory leak
class Objs {
public:
    std::vector<Obj*> objects;
private:
    // better way of doing this?
    Obj* closestObj = nullptr;
    float minDist = std::numeric_limits<float>::max();
    float dist = -1.0f;
public:
    Objs(std::vector<Plane*> p,std::vector<Sphere*> s){
        for(auto it=p.begin();it!=p.end();++it){
            objects.push_back(*it);
        }
        for(auto it=s.begin();it!=s.end();++it){
            objects.push_back(*it);
        }
    }

    // return the closest object as well as the distance along the ray
    std::tuple<Obj*,float> findFirstObject(const Ray& r){
        // reset variables from last call
        closestObj = nullptr;
        minDist = std::numeric_limits<float>::max();
        dist = -0.1f;

        // for every object in scene
        for(auto it=objects.begin();it!=objects.end();++it){

            // this will be t in x=o+td equation of line
            // each intersects() method will return -1 if there
            // is no VALID intersection (positive, not infinite etc)
            dist = (*it)->intersects(r);

            if (0.0f < dist && dist < minDist) {
                // then it is the closest object so far, so store
                // the dist and the reference to the object
                minDist = dist;
                closestObj = *it;
            }
        }
        // closestObj will still be nullptr if the ray didn't hit anything
        return std::make_tuple(closestObj,minDist);
    }
};

class LightSource {
public: // properties
    Vec3 position = Vec3(0,0,0);
    Colour ambient = Colour(0.0f, 0.0f, 0.0f);
    Colour diffuse = Colour(0.0f, 0.0f, 0.0f);
    Colour specular = Colour(0.0f, 0.0f, 0.0f);
private: // for calculations
    Vec3 normal = Vec3(0,0,0);
    Vec3 lightAngle = Vec3(0,0,0);
    Vec3 R = Vec3(0,0,0);
    Colour Ia = Colour(0.0f, 0.0f, 0.0f);
    Colour Id = Colour(0.0f, 0.0f, 0.0f);
    Colour Is = Colour(0.0f, 0.0f, 0.0f);
    Ray shadow;
    float shadowLength = 0.0f;
    float interLength = 0.0f;
    Obj* obj; // this is not used but must be there for findFirstObject()
public:

    LightSource (Vec3 pos, Colour a,Colour d,Colour s) {
        position = pos;
        ambient = a;
        diffuse = d;
        specular = s;
        printf("Light Source: ");
        printVec(position);
    }

    // ensure all light levels fall in correct range
    void bindValues(){
        Ia = bindPixelValues(Ia);
        // if diffuse <= 0, specular must be 0
        for(int i=0;i<3;i++){ if(Id(i)<0.0f){ Is(i)=0.0f;}}
        Id = bindPixelValues(Id);
        Is = bindPixelValues(Is);
    }

    // determine the colour of a particular pixel
    // use Phong shading method
    // use shadow rays
    //
    Colour illuminate(Objs objs,const Obj* o,const Ray& r,const Vec3& intersection){
        // ambient light calculation
        Ia = mult(o->ka,ambient);

        // create ray from intersection to light source
        // and store the distance between those points
        shadowLength = shadow.createBetween(intersection,position,0.01f);
        // check for objects along the shadow ray
        std::tie(obj,interLength) = objs.findFirstObject(shadow);
        // if there is an object before the light, don't calculate Id or Is
        //if (false){
        if (interLength < shadowLength){
            Id = Colour(0.0f, 0.0f, 0.0f);
            Is = Colour(0.0f, 0.0f, 0.0f);
        } else {
            // diffused light calculation
            normal = o->normal(intersection);
            lightAngle = (position-intersection).normalized();
            Id = mult(lightAngle.dot(normal)*o->kd,diffuse);

            // specular light calculation
            R = 2*lightAngle.dot(normal)*normal-lightAngle;
            Is = mult(o->ks*pow(R.dot(-r.dir),o->alpha),specular);
        }
        bindValues();
        return (Ia + Id + Is);
    }
};



// sets up the camera and image plane and optionally switched the x-axis
// to be positive to the right if that is more intuitive
void setup(Camera& cam, ImagePlane& im, bool swapXAxis,std::vector<Plane*> &planes,
           std::vector<Sphere*> &spheres, LightSource& L){

    if (swapXAxis) {
        cam.position    = mult(cam.position, Vec3(-1,1,1));
        cam.lookingAt   = mult(cam.lookingAt,Vec3(-1,1,1));
        L.position = mult(L.position,Vec3(-1,1,1));
        for(auto it=planes.begin();it!=planes.end();++it){
            (*it)->point = mult((*it)->point, Vec3(-1,1,1));
            (*it)->norm = mult((*it)->norm, Vec3(-1,1,1));
        }
        for(auto it=spheres.begin();it!=spheres.end();++it){
            (*it)->center = mult((*it)->center, Vec3(-1,1,1));
        }
    }
    cam.setup();
    im.setup(cam);
}

int main(int, char**){
    // (+x: left +y: up +z: out of screen)

    // DEFINE CAMERA
    // {position,lookingAt}
    Camera cam = {Vec3(0,0,4),Vec3(0,0,0)};

    // DEFINE IMAGE PLANE
    // default is 640x480 w/ 90 deg view angle
    ImagePlane im;

    // DEFINE OBJECTS IN SCENE

    // PLANES
    // Plane p(point, normal, ambient, diffuse, specular, alpha)
    // normal can be any length and it will be normalized during construction
    Plane floor  (Vec3(0,-1,0),Vec3(0,1,0), Colour(.9f,.9f,.9f),Colour(.9f,.9f,.9f),Colour(.9f,.9f,.9f),2);
    Plane ceiling(Vec3(0,1,0), Vec3(0,1,0), Colour(.9f,.9f,.9f),Colour(.9f,.9f,.9f),Colour(.9f,.9f,.9f),2);
    Plane left   (Vec3(-1,0,0),Vec3(1,0,0), Colour(.4f,.4f,.4f),Colour(.4f,.4f,.4f),Colour(.4f,.4f,.4f),2);
    Plane right  (Vec3(1,0,0), Vec3(1,0,0), Colour(.4f,.4f,.4f),Colour(.4f,.4f,.4f),Colour(.4f,.4f,.4f),2);
    Plane back   (Vec3(0,0,-1),Vec3(0,0,1), Colour(.2f,.2f,.7f),Colour(.2f,.2f,.7f),Colour(.2f,.2f,.7f),2);

    std::vector<Plane*> planes;

    planes.push_back(&floor);
    //planes.push_back(&ceiling);
    //planes.push_back(&left);
    //planes.push_back(&right);
    //planes.push_back(&back);

    // SPHERES
    // Sphere s(position,radius,ambient,diffuse,specular,alpha)
    Sphere s1   (Vec3(0,0,0),1,   Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),5);
    Sphere s2   (Vec3(2,2,1), 1,   Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),5);
    Sphere s3   (Vec3(-2,-.5f,-.5f), .5f,   Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f),5);

    std::vector<Sphere*> spheres;

    spheres.push_back(&s1);
    //spheres.push_back(&s2);
    spheres.push_back(&s3);

    // DEFINE LIGHTING SOURCES
    // {position,ambient,diffuse,specular}
    LightSource L(Vec3(-4,4,4),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f));

    bool swapXAxis = true;
    setup(cam,im,swapXAxis,planes,spheres,L);

    // INITIALIZE LOOP VARIABLES
    Ray pixel;
    Objs objects = Objs(planes,spheres);
    Obj* closestObject = nullptr;
    float distance = 0.0f;

    // add test for image plane intersecting any objects?


    for (int row = 0; row < im.image.rows(); ++row) {
        for (int col = 0; col < im.image.cols(); ++col) {

            pixel.constructPrimary(im,row,col,cam);
            // get both the object and the distance
            std::tie(closestObject,distance) = objects.findFirstObject(pixel);

            if (closestObject != nullptr) {
                im.image(row,col) = L.illuminate(objects,closestObject,pixel,pixel.at(distance));
            } // else the pixel stays black
        }
    }

    bmpwrite("../../out.bmp", im.image);
    imshow(im.image);

    return EXIT_SUCCESS;
}
