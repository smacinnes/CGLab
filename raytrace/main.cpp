#include "OpenGP/Image/Image.h"
#include "bmpwrite.h"

using namespace OpenGP;

using Colour = Vec3; // RGB Value
Colour red()    { return Colour(1.0f, 0.0f, 0.0f); }
Colour green()  { return Colour(0.0f, 1.0f, 0.0f); }
Colour blue()   { return Colour(0.0f, 0.0f, 1.0f); }
Colour white()  { return Colour(1.0f, 1.0f, 1.0f); }
Colour black()  { return Colour(0.0f, 0.0f, 0.0f); }

// colours should be expressed 0-1 so modify this or divide answer by 256
float bindPixel(float value){
    if (value <= 0)    return 0.0f;
    if (value >= 1.0f) return 1.0f;
    return value;
}
Vec3 bindPixelValues(const Vec3& v){
    return Vec3(bindPixel(v(0)),bindPixel(v(1)),bindPixel(v(2)));
}
// multiply two vectors elementwise - ex for colours
Vec3 mult(const Vec3& v1,const Vec3& v2){
    return Vec3(v1(0)*v2(0),v1(1)*v2(1),v1(2)*v2(2));
}
// multiply a vector by a scaler - is there really no function for this already?
Vec3 mult(const Vec3& v,float s){
    return Vec3(v(0)*s,v(1)*s,v(2)*s);
}
Vec3 mult(float s,const Vec3& v){
    return Vec3(v(0)*s,v(1)*s,v(2)*s);
}
void printVec(const Vec3& v){
    printf("%.3f  %.3f  %.3f",double(v(0)),double(v(1)),double(v(2)));
}
void printlnVec(const Vec3& v){
    printf("%.3f  %.3f  %.3f\n",double(v(0)),double(v(1)),double(v(2)));
}

/*
 *  The point (0,0,0) will be an absolute position in space and is not defined
 *  relative to the camera, image plane or anything else in the scene. The
 *  distance from the camera to the image plane is defined as 1.0 (but this
 *  can be changed under ImagePlane) and all other dimensions will be modeled
 *  off of that distance.
 */

class Camera {
public:
    // set these three manually
    Vec3 position =  Vec3(0,0,5);   // location of body of camera
    Vec3 lookingAt = Vec3(0,0,4);   // where the camera is facing
    Vec3 up = Vec3(0,1,0);          // tilt of camera (0,1,0) is no tilt - add easier adjustment
    // these will be set during setup() call
    Vec3 viewingDir = Vec3(0,0,0);
    Vec3 rightAxis = Vec3(0,0,0);
    Vec3 upAxis = Vec3(0,0,0);

    void setup(){
        viewingDir = lookingAt - position;
        viewingDir.normalize();
        rightAxis = up.cross(viewingDir);
        rightAxis.normalize();
        upAxis = viewingDir.cross(rightAxis);
    }
};

class ImagePlane {
private:
    // set these manually
    int wResolution = 640;      // default
    int hResolution = 480;      // default
    float viewingAngle = 90.0f; // recommended - most natural for humans
    float distToCam = 1.0f;     // all other dimensions based off this
    // rest are calculated automatically
    Vec3 center = Vec3(0,0,0);
    Vec3 corner = Vec3(0,0,0);
    float hwRatio = float(hResolution)/wResolution;
    float halfWidth = 0.0f;

public:
    Image<Colour> image = Image<Colour>(hResolution, wResolution);
    Vec3 llc = Vec3(0,0,0);   // lower left corner
    Vec3 pixRi = Vec3(0,0,0); // move one pixel to the right
    Vec3 pixUp = Vec3(0,0,0); // move one pixel up

    void setup(const Camera& c){
        center = c.position+mult(c.viewingDir,distToCam);
        halfWidth = distToCam*tanf(viewingAngle/2.0f);
        llc = center - mult(c.rightAxis,halfWidth) - mult(c.upAxis,halfWidth*hwRatio);
        pixRi = mult(2*halfWidth/(wResolution-1),c.rightAxis);
        pixUp = mult(2*halfWidth*hwRatio/(hResolution-1),c.upAxis);
    }
};

class Sphere {
public:
    Vec3 center = Vec3(0,0,0);
    float radius = 0.0f;
    Colour ka = Colour(0.0f, 0.0f, 0.0f);
    Colour kd = Colour(0.0f, 0.0f, 0.0f);
    Colour ks = Colour(0.0f, 0.0f, 0.0f);
    float alpha = 0.0f;
};

class Ray {
public:
    Vec3 orig = Vec3(0,0,0);
    Vec3 dir = Vec3(0,0,0);
private:
    Vec3 co = Vec3(0,0,0);      //center of object to origin of ray
    float disc = 0.0f;
    float t = 0.0f;
    float b = 0.0f;
public:
    // intersect image plane at specified pixel
    void constructPrimary(const ImagePlane& im,int row,int col,const Camera& cam){
        orig = im.llc + mult(im.pixRi,col-1) + mult(im.pixUp,row);
        // pixel direction as unit vector
        dir = orig - cam.position;
        dir.normalize();
    }
    // if the ray intersects the given sphere
    bool intersects(const Sphere& s){
        co = orig-s.center;
        disc = powf(dir.dot(co),2)-(powf(co.norm(),2)-powf(s.radius,2));
        return disc >= 0.0f;
    }
    // assumes intersects() has already been called
    // add error message printouts for invalid disc and t?
    Vec3 findSphereIntersection(){
        if (disc < 0) return Vec3(0,0,0);
        b = -dir.dot(co);
        t = fmax(fmin(b+sqrt(disc),b-sqrt(disc)),0.0f);
        return orig + t*dir;
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

    Colour illuminate(const Sphere& s,const Ray& r,const Vec3& intersection){
        // ambient light calculation
        Ia = bindPixelValues(mult(s.ka,ambient));

        // diffused light calculation
        normal = intersection - s.center;
        normal.normalize();
        lightAngle = intersection - position;
        lightAngle.normalize();
        Id = mult(mult(s.kd,float(lightAngle.dot(normal))),diffuse);

        // specular light calculation
        R = mult(2*lightAngle.dot(normal),normal)-lightAngle;
        Is = mult(mult(s.ks,pow(R.dot(-r.dir),s.alpha)),specular);

        bindValues();
        return (Ia + Id + Is);
    }
};



int main(int, char**){

    /* PROBLEMS
     * when moving objects around scene, x always seems reversed except when moving
     * light source, in which case x is fine and the other directions seem reversed
     */
    // DEFINE CAMERA
    Camera cam = {Vec3(0,0,10),Vec3(0,0,-10)}; // {position,lookingAt}
    cam.setup();

    // DEFINE IMAGE PLANE
    ImagePlane im;
    im.setup(cam);

    // DEFINE OBJECTS IN SCENE (+x: right +y: up +z: close)
    Sphere s = {Vec3(0,0,2),1,Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),5};

    // DEFINE LIGHTING SOURCES
    LightSource L = {Vec3(0,0,-5),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f)};

    // INITIALIZE LOOP VARIABLES
    Ray pixel;
    Vec3 intersection = Vec3(0,0,0);

    // test for image plane intersecting any objects?


    for (int row = 0; row < im.image.rows(); ++row) {
        for (int col = 0; col < im.image.cols(); ++col) {

            pixel.constructPrimary(im,row,col,cam);

            if (pixel.intersects(s)) {
                intersection = pixel.findSphereIntersection();

                im.image(row,col) = L.illuminate(s,pixel,intersection);

            } else {
                im.image(row,col) = black();
            }


        }
    }

    bmpwrite("../../out.bmp", im.image);
    imshow(im.image);

    return EXIT_SUCCESS;
}

