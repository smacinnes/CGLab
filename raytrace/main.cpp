/*
 * Seamus MacInnes
 * A00415673
 * CSCI 4471 Computer Graphics
 * Assignment 1: Ray Tracing
 * Oct 8th 2019
 */

#include "OpenGP/Image/Image.h"
#include "bmpwrite.h"

using namespace OpenGP;

using Colour = Vec3; // RGB Value
Colour red()    { return Colour(1.0f, 0.0f, 0.0f); }
Colour green()  { return Colour(0.0f, 1.0f, 0.0f); }
Colour blue()   { return Colour(0.0f, 0.0f, 1.0f); }
Colour white()  { return Colour(1.0f, 1.0f, 1.0f); }
Colour black()  { return Colour(0.0f, 0.0f, 0.0f); }

// Accepted tolerance for floating point numbers
// used for ray offsets and zero dot products
const float TOLERANCE = 0.001f;

// multiply two vectors elementwise (useful for colours)
Vec3 mult(const Vec3& v1,const Vec3& v2){
    return Vec3(v1(0)*v2(0),v1(1)*v2(1),v1(2)*v2(2));
}
// if two floats are close enough
bool equal(float a, float b, float e){
    return std::abs(a-b) <= e;
}

/*  EXPLANATION OF COORDINATE SYSTEM
 *  The point (0,0,0) will be an absolute position in space and is not defined
 *  relative to the camera, image plane or anything else. The distance from
 *  the camera to the image plane is defined as 1.0 for simplicity (but this
 *  can be changed in the ImagePlane constructor).
 */

// Class containing all information about the camera
class Camera {
public:
    // set these
    Vec3 position,lookingAt,up;        // up = (0,1,0) is no tilt
    // calculated automatically
    Vec3 viewingDir,rightAxis,upAxis;

    Camera(const Vec3& pos,const Vec3& look,const Vec3& u) :
        position(pos),
        lookingAt(look),
        up(u)
    {   // calculate camera coordinate frame
        viewingDir = (lookingAt - position).normalized();
        rightAxis = (viewingDir.cross(up)).normalized();
        upAxis = rightAxis.cross(viewingDir); // will be unit vec
    }
};

// Class defining all properties of the image plane
// Image plane is defined relative to the camera
class ImagePlane {
private:
    // set these manually
    int wResolution = 640;      // default is 640
    int hResolution = 480;      // default is 480
    float viewingAngle = 80.0f; // in degrees - 80 appears natural
    float distToCam = 1.0f;
    // these are calculated automatically
    Vec3 center;
    float hwRatio,halfWidth;
public:
    Image<Colour> image = Image<Colour>(hResolution, wResolution);
    Vec3 llc,pixRi,pixUp;

    ImagePlane(int wRes,int hRes,float viewAng,float camDist,const Camera& c) :
        wResolution(wRes),
        hResolution(hRes),
        viewingAngle(viewAng),
        distToCam(camDist)
    {   // calculate plane location, orientation and boundaries
        center = c.position+c.viewingDir*distToCam;
        hwRatio = float(hResolution)/wResolution;
        halfWidth = distToCam*tanf(viewingAngle*float(M_PI)/360.0f);
        llc = center - c.rightAxis*halfWidth - c.upAxis*halfWidth*hwRatio;
        pixRi = 2*halfWidth/(wResolution-1)*c.rightAxis;
        pixUp = 2*halfWidth*hwRatio/(hResolution-1)*c.upAxis;
    }
};

class Ray {
public:
    Vec3 orig,dir;

    /* create a Ray between these two points
     * offset is distance along ray to move
     * the start positionto avoid self intersection
     * returns the distance between the points */
    float createBetween(const Vec3& from, const Vec3& to, float offset){
        dir = (to-from).normalized();
        orig = from+offset*dir;
        return (to-orig).norm();
    }
    // create a Ray from origin and direction with an offset
    void createWith(const Vec3& o, const Vec3& d, float offset){
        dir = d.normalized();
        orig = o+offset*dir;
    }
    // create Ray intersecting ImagePlane at specified pixel
    void constructPrimary(const ImagePlane& im,int row,int col,const Camera& cam){
        orig = cam.position;
        dir = (im.llc+im.pixRi*float(col-1)+im.pixUp*float(row-1)
               -cam.position).normalized();
    }
    // return Ray location at given distance x=o+t*d
    Vec3 at(float t) const {
        return orig+t*dir;
    }
};

// All objects in the scene will derive from this class and
// have these material properties and methods
class Obj {
public:
    Colour ka,kd,ks,ksh;
    float alpha;
    bool isShiny;

    Obj(const Vec3& a,
        const Vec3& d,
        const Vec3& s,
        const Vec3& sh,
        float al) :
        ka(a),
        kd(d),
        ks(s),
        ksh(sh),
        alpha(al){
        // if object reflects any light - saves calculation time
        isShiny = sh(0) > 0 || sh(1) > 0 || sh(2) > 0;
    }

    // returns distance to valid intersection if one exists, -1 otherwise
    virtual float intersects(const Ray&) = 0;

    // returns the normal at the specified point
    virtual Vec3 normal(const Vec3&) const = 0;

    // compiler complains without virtual destructor
    virtual ~Obj() {}
};

// define a triangle using 3 points
class Triangle : public Obj {
public:
    Vec3 v1,v2,v3,edge1,edge2,norm;
private:
    Vec3 h,s,q;
    float a,f,u,v,t;
public:
    Triangle(const Vec3& va,
             const Vec3& vb,
             const Vec3& vc,
             const Vec3& a,
             const Vec3& d,
             const Vec3& s,
             const Vec3& sh,
             float al) :
        Obj(a,d,s,sh,al),
        v1(va),
        v2(vb),
        v3(vc)
    {
        edge1 = v2 - v1;
        edge2 = v3 - v1;
        norm = edge1.cross(edge2).normalized();
    }

    /* returns the distance along the ray from the triangle to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning a single positive solution inside the triangle
     * using the Moller-Trumbore algorithm */
    float intersects(const Ray& r){
        h = r.dir.cross(edge2);
        a = edge1.dot(h);
        if (equal(a,0.0f,TOLERANCE)) return -1.0f;    // parallel
        f = 1.0f/a;
        s = r.orig - v1;
        u = f*s.dot(h);
        if (u < 0.0f || u > 1.0f) return -1.0f;     // outside triangle
        q = s.cross(edge1);
        v = f*r.dir.dot(q);
        if (v < 0.0f || u+v > 1.0f) return -1.0f;   // outside triangle
        t = f*edge2.dot(q);
        if (t > 0.0f) return t;                     // valid intersection
        return -1.0f;                               // triangle behind ray
    }
    // return normal (need param for overloading)
    Vec3 normal(const Vec3&) const {
        return norm;
    }
};

// defines an infinite plane (ex the floor)
class Plane : public Obj{
public:
    Vec3 point,norm;
private:
    float temp;
public:
    Plane(const Vec3& p,
          const Vec3& n,
          const Vec3& a,
          const Vec3& d,
          const Vec3& s,
          const Vec3& sh,
          float al) :
        Obj(a,d,s,sh,al),
        point(p),
        norm(n.normalized()){}

    /* returns the distance along the ray from the plane to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning a single positive solution */
    float intersects(const Ray& r){
        temp = norm.dot(r.dir);                         // hold dot prod op
        if(equal(temp,0.0f,TOLERANCE)) return -1.0f;    // if dot~=0
        temp = (point-r.orig).dot(norm)/temp;           // hold scalar solution
        if (temp > 0) return temp;                      // ensure (+) solution
        return -1.0f;
    }
    // return normal (need param for overloading)
    Vec3 normal(const Vec3&) const {
        return norm;
    }
};

// defines a sphere via a point and a radius
class Sphere : public Obj {
public:
    Vec3 center;
    float radius;
private:
    Vec3 co;
    float disc,t1,t2,b;
public:
    Sphere(const Vec3& c,
           float r,
           const Vec3& a,
           const Vec3& d,
           const Vec3& s,
           const Vec3& sh,
           float al) :
        Obj(a,d,s,sh,al),
        center(c),
        radius(r){}

    /* returns the distance along the ray from the sphere to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning that at least one solution is positive
     * two pos:    ray starts before sphere    (return smallest solution)
     * one pos:    ray starts inside sphere    (cross section - optional)
     * two neg:    ray starts after sphere     (ignore)
     * no real:    ray does not intersect      (ignore) */
    float intersects(const Ray& r){
        co = r.orig-center;
        disc = powf(r.dir.dot(co),2)-powf(co.norm(),2)+powf(radius,2);
        if (disc < 0.0f) return -1.0f; // complex solutions, no intersection
        b = -(r.dir.dot(co));
        t1 = b+sqrt(disc);
        t2 = b-sqrt(disc);
        if (t1 < 0.0f) return -1.0f;       // ray starts after sphere
        // to avoid cross sections, change the previous line to read "t2" and
        // comment out the following line
        if (t2 < 0.0f) return t1;
        return t2;
    }
    // return normal
    Vec3 normal(const Vec3& p) const {
        return (p-center).normalized();
    }
};

// contains vector of references to all scene objects
// and determines first object a ray intersects with
class Objs {
public:
    std::vector<Obj*> objects;
private:
    Obj* closestObj = nullptr;
    float minDist = std::numeric_limits<float>::max();
    float dist = -1.0f;
public:
    Objs(const std::vector<Plane*>& p,
         const std::vector<Sphere*>& s,
         const std::vector<Triangle*>& t){
        for(auto it=p.begin();it!=p.end();++it){
            objects.push_back(*it);
        }
        for(auto it=s.begin();it!=s.end();++it){
            objects.push_back(*it);
        }
        for(auto it=t.begin();it!=t.end();++it){
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
        // minDist will still be maximim float distance
        return std::make_tuple(closestObj,minDist);
    }
};

// contains all lighting information and calculates pixel values
class LightSource {
public:
    Vec3 position;
    Colour ambient,diffuse,specular;
    int maxDepth;
private:
    Vec3 normal,lightAngle,R,intersection;
    Colour background,pixelColour;
    Ray shadow,reflect;
    Obj *obj,*Sobj;
    float distToLight,interLength;
public:
    LightSource (const Vec3& pos,
                 const Colour& a,
                 const Colour& d,
                 const Colour& s,
                 int depth) :
        position(pos),
        ambient(a),
        diffuse(d),
        specular(s),
        maxDepth(depth){
        background = Colour(0.0f,0.0f,0.0f); // if doesn't hit anything
    }
    // bind a single pixel value in range [0.0f,1.0f]
    float bindPixel(float value){
        if (value <= 0.0f) return 0.0f;
        if (value >= 1.0f) return 1.0f;
        return value;
    }
    // bind a all pixel values in range [0.0f,1.0f]
    Vec3 bindVec(Vec3 v){
        return Vec3(bindPixel(v(0)),bindPixel(v(1)),bindPixel(v(2)));
    }
    // determine the colour of a particular pixel
    // use Phong shading method
    // use shadow rays
    // optional reflections
    Colour illuminate(Objs& objs,
                      const Ray& r,
                      int recursionDepth){

        // if it doesn't hit anything, this will be the colour
        if (recursionDepth == 0) pixelColour = background;

        // determine if ray intersects anything
        std::tie(obj,interLength) = objs.findFirstObject(r);

        // if it hit something
        if (interLength > 0) {

            // calculate intersection and normal at point
            intersection = r.at(interLength);
            normal = obj->normal(intersection);

            // ambient light calculation
            pixelColour += bindVec(mult(obj->ka,ambient));

            // check if it's in a shadow
            // return the dist to the light source along the ray
            distToLight = shadow.createBetween(intersection,position,TOLERANCE);
            std::tie(Sobj,interLength) = objs.findFirstObject(shadow);

            // only calculate diffuse + specular if not in shadow
            if (interLength > distToLight){
                // diffused light calculation
                lightAngle = (position-intersection).normalized();
                pixelColour += bindVec(mult(lightAngle.dot(normal)*obj->kd,diffuse));

                // specular light calculation
                R = 2*lightAngle.dot(normal)*normal-lightAngle;
                pixelColour += bindVec(mult(obj->ks*pow(R.dot(-r.dir),obj->alpha),specular));
            }

            // reflection calculations
            if (recursionDepth < maxDepth) {
                if (obj->isShiny) {
                    // create a reflection of the view vector
                    reflect.createWith(intersection,2*(-r.dir).dot(normal)*normal+r.dir,TOLERANCE);
                    //printVec(pixelColour);
                    pixelColour += bindVec(mult(obj->ksh,illuminate(objs,reflect,recursionDepth+1)));
                    //printVec(pixelColour);
                }
            }
        }
        return bindVec(pixelColour);
    }
};

// ensure the normals of planes and triangles are facing the in the right direction
// otherwise shadows will not appear
void checkNormals(Camera& cam,
           const std::vector<Plane*> &planes,
           const std::vector<Triangle*> &triangles){

    // check the normal of planes and triangles against the camera
    // if the normals don't face the camera, reverse them
    for(auto it=planes.begin();it!=planes.end();++it){
        if ((*it)->norm.dot(cam.position-(*it)->point) < 0.0f)
            (*it)->norm *= -1.0f;
    }
    for(auto it=triangles.begin();it!=triangles.end();++it){
        if ((*it)->norm.dot(cam.position-(*it)->v1) < 0.0f)
            (*it)->norm *= -1.0f;
    }
}

int main(int, char**){
    // positive axes are (x-right, y-up, z-out of screen)

    // DEFINE CAMERA
    // (position,lookingAt,upAxis)
    Camera cam(Vec3(1.5,0,4),Vec3(0,0,0),Vec3(0,1,0));

    // DEFINE IMAGE PLANE
    // (width,height,viewAngle(deg),distFromCamera,camera)
    ImagePlane im(640,480,80,1,cam);

    // DEFINE OBJECTS IN SCENE

    // PLANES
    // if needed, passed normal vector will be scaled or reversed
    Plane floor  (Vec3(0,-2,0),         // point
                  Vec3(0,-1,0),         // normal
                  Colour(.9f,.9f,.9f),  // ambient
                  Colour(.9f,.9f,.9f),  // diffuse
                  Colour(.9f,.9f,.9f),  // specular
                  Colour(-1,-1,-1),     // reflection
                  2);                   // alpha (for specular)
    Plane ceiling(Vec3(0,2,0),
                  Vec3(0,1,0),
                  Colour(.9f,.9f,.9f),
                  Colour(.9f,.9f,.9f),
                  Colour(.9f,.9f,.9f),
                  Colour(-1,-1,-1),
                  2);
    Plane left   (Vec3(-2,0,0),
                  Vec3(-1,0,0),
                  Colour(.6f,.2f,.2f),
                  Colour(.6f,.2f,.2f),
                  Colour(.6f,.2f,.2f),
                  Colour(-1,-1,-1),
                  2);
    Plane right  (Vec3(2,0,0),
                  Vec3(1,0,0),
                  Colour(.2f,.6f,.2f),
                  Colour(.2f,.6f,.2f),
                  Colour(.2f,.6f,.2f),
                  Colour(-1,-1,-1),
                  2);
    Plane back   (Vec3(0,0,-2),
                  Vec3(0,0,-1),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f),
                  Colour(-1,-1,-1),
                  2);
    Plane front   (Vec3(0,0,4.5f),
                  Vec3(0,0,-1),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f),
                  Colour(-1,-1,-1),
                  2);

    // only objects added to appropriate vector will appear in scene
    std::vector<Plane*> planes;
    planes.push_back(&floor);
    planes.push_back(&ceiling);
    planes.push_back(&left);
    planes.push_back(&right);
    planes.push_back(&back);
    planes.push_back(&front);

    // SPHERES
    Sphere s1   (Vec3(0.5f,0,0),        // center
                 1,                     // radius
                 Colour(.6f,.2f,.6f),   // ambient
                 Colour(.6f,.2f,.6f),   // diffuse
                 Colour(.6f,.2f,.6f),   // specular
                 Colour(1,1,1),         // reflections
                 5);                    // alpha
    Sphere s2   (Vec3(-1.5f,-.5f,1),
                 .5f,
                 Colour(.1f,.1f,.6f),
                 Colour(.1f,.1f,.6f),
                 Colour(.1f,.1f,.6f),
                 Colour(.5,.5,.5),
                 5);

    std::vector<Sphere*> spheres;
    spheres.push_back(&s1);
    spheres.push_back(&s2);

    // TRIANGLES
    Triangle t1(Vec3(0,-1,0),           // vertex 1
                Vec3(1,1,0),            // vertex 2
                Vec3(1,-1,0),           // vertex 3
                Colour(.9f,.9f,.9f),    // ambient
                Colour(.9f,.9f,.9f),    // diffuse
                Colour(.9f,.9f,.9f),    // specular
                Colour(-1,-1,-1),       // reflects
                2);                     // alpha

    std::vector<Triangle*> triangles;
    //triangles.push_back(&t1);

    // DEFINE LIGHTING SOURCE
    LightSource L(Vec3(0,1.9f,-.5),     // position
                  Colour(.6f,.6f,.6f),  // ambient
                  Colour(.6f,.6f,.6f),  // diffuse
                  Colour(.4f,.4f,.4f),  // specular
                  4);                   // recursion depth

    // verify normals face camera
    checkNormals(cam,planes,triangles);

    // INITIALIZE LOOP VARIABLES
    Ray pixel;
    Objs objects = Objs(planes,spheres,triangles);

    // ILLUMINATE ALL PIXELS
    for (int row = 0; row < im.image.rows(); ++row) {
        for (int col = 0; col < im.image.cols(); ++col) {

            pixel.constructPrimary(im,row,col,cam);

            im.image(row,col) = L.illuminate(objects,pixel,0);
        }
    }

    // display and save the image
    bmpwrite("../../out.bmp", im.image);
    imshow(im.image);

    return EXIT_SUCCESS;
}
