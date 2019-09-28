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
Vec3 elmult(const Vec3& v1,const Vec3& v2){
    return Vec3(v1(0)*v2(0),v1(1)*v2(1),v1(2)*v2(2));
}
// multiply a vector by a scaler - is there really no function for this already?
Vec3 elmult(const Vec3& v,float s){
    return Vec3(v(0)*s,v(1)*s,v(2)*s);
}
Vec3 elmult(float s,const Vec3& v){
    return Vec3(v(0)*s,v(1)*s,v(2)*s);
}
void printVec(const Vec3& v){
    printf("%.3f  %.3f  %.3f",double(v(0)),double(v(1)),double(v(2)));
}
void printlnVec(const Vec3& v){
    printf("%.3f  %.3f  %.3f\n",double(v(0)),double(v(1)),double(v(2)));
}

class Camera {
public:
    Vec3 position = Vec3(0,0,0);
};

class ImagePlane {
    // distance units are defined respective to the image plane
    // half the height of the plane is defined as length one
private:
    int wResolution = 640;      // default
    int hResolution = 480;      // default
    float viewingAngle = 90.0f; // recommended
    float whRatio = float(wResolution)/hResolution;
public:
    Image<Colour> image = Image<Colour>(hResolution, wResolution);
    float distFromCam = -whRatio/tanf(viewingAngle/2.0f);                   // (-) is in front of camera
    Vec3 llc = Vec3(-whRatio,-1,distFromCam);   // lower left corner
    Vec3 urc = Vec3( whRatio, 1,distFromCam);   // upper right corner
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
    Vec2 inPlane = Vec2(0,0);   // intersection wrt width/height [0,1]
    Vec3 co = Vec3(0,0,0);      //center of object to origin of ray
    float disc = 0.0f;
    float t = 0.0f;
    float b = 0.0f;
public:
    // intersect image plane at specified pixel
    void constructPrimary(const ImagePlane& im,int row,int col,const Camera& cam){
        inPlane = Vec2((col+0.5)/im.image.cols(),(row+0.5)/im.image.rows());
        orig = Vec3((im.urc(0)-im.llc(0))*inPlane(0)+im.llc(0),
                    (im.urc(1)-im.llc(1))*inPlane(1)+im.llc(1),
                     im.distFromCam);
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
        Ia = bindPixelValues(elmult(s.ka,ambient));

        // diffused light calculation
        normal = intersection - s.center;
        normal.normalize();
        lightAngle = intersection - position;
        lightAngle.normalize();
        Id = elmult(elmult(s.kd,float(lightAngle.dot(normal))),diffuse);

        // specular light calculation
        R = elmult(2*lightAngle.dot(normal),normal)-lightAngle;
        Is = elmult(elmult(s.ks,pow(R.dot(-r.dir),s.alpha)),specular);

        bindValues();
        return (Ia + Id + Is);
    }
};



int main(int, char**){

    // DEFINE CAMERA
    Camera cam;

    // DEFINE IMAGE PLANE
    ImagePlane im;

    // DEFINE OBJECTS IN SCENE (+x: right +y: up +z: close)
    Sphere s = {Vec3(0,0,-3),1,Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),Colour(.6f,.2f,.6f),5};

    // DEFINE LIGHTING SOURCES
    LightSource L = {Vec3(-5,-5,-5),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f),Colour(.6f,.6f,.6f)};

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

