#include "OpenGP/Image/Image.h"
#include "bmpwrite.h"
#include "obj.h"
#include "lightsource.h"
#include "plane.h"
#include "sphere.h"
#include "triangle.h"
#include "objs.h"

using namespace OpenGP;

using Colour = Vec3; // RGB Value
Colour red()    { return Colour(1.0f, 0.0f, 0.0f); }
Colour green()  { return Colour(0.0f, 1.0f, 0.0f); }
Colour blue()   { return Colour(0.0f, 0.0f, 1.0f); }
Colour white()  { return Colour(1.0f, 1.0f, 1.0f); }
Colour black()  { return Colour(0.0f, 0.0f, 0.0f); }

// multiply two vectors elementwise (useful for colours)
Vec3 mult(const Vec3& v1,const Vec3& v2){
    return Vec3(v1(0)*v2(0),v1(1)*v2(1),v1(2)*v2(2));
}

// sets up the camera and image plane and optionally switched the x-axis
// to be positive to the right if that is more intuitive
void setup(Camera& cam,
           ImagePlane& im,
           bool posXright,
           const std::vector<Plane*> &planes,
           const std::vector<Sphere*> &spheres,
           const std::vector<Triangle*> &triangles,
           LightSource& L){

    // if the user prefers the x-axis positive to the right,
    // mirror all coordinates
    if (posXright) {
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
        for(auto it=triangles.begin();it!=triangles.end();++it){
            (*it)->v1 = mult((*it)->v1, Vec3(-1,1,1));
            (*it)->v2 = mult((*it)->v2, Vec3(-1,1,1));
            (*it)->v3 = mult((*it)->v3, Vec3(-1,1,1));
            (*it)->edge1 = mult((*it)->edge1, Vec3(-1,1,1));
            (*it)->edge2 = mult((*it)->edge2, Vec3(-1,1,1));
        }
    }
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
    cam.setup();
    im.setup(cam);
}

int main(int, char**){
    // (+x: left +y: up +z: out of screen)

    // DEFINE CAMERA
    // (position,lookingAt,upAxis)
    Camera cam(Vec3(0,0,4),Vec3(0,0,0),Vec3(0,1,0));

    // DEFINE IMAGE PLANE
    // (wRes,hRes,viewAngle,distToCam)
    ImagePlane im(640,480,90.0f,1.0f);

    // DEFINE OBJECTS IN SCENE

    // PLANES
    // Plane p(point, normal, ambient, diffuse, specular, alpha)
    // normal can be any length and it will be normalized during construction
    // normal may be reversed to ensure correct lighting
    Plane floor  (Vec3(0,-2,0),
                  Vec3(0,-1,0),
                  Colour(.9f,.9f,.9f),
                  Colour(.9f,.9f,.9f),
                  Colour(.9f,.9f,.9f),
                  2);
    Plane ceiling(Vec3(0,2,0),
                  Vec3(0,1,0),
                  Colour(.9f,.9f,.9f),
                  Colour(.9f,.9f,.9f),
                  Colour(.9f,.9f,.9f),
                  2);
    Plane left   (Vec3(-2,0,0),
                  Vec3(-1,0,0),
                  Colour(.4f,.4f,.4f),
                  Colour(.4f,.4f,.4f),
                  Colour(.4f,.4f,.4f),
                  2);
    Plane right  (Vec3(2,0,0),
                  Vec3(1,0,0),
                  Colour(.4f,.4f,.4f),
                  Colour(.4f,.4f,.4f),
                  Colour(.4f,.4f,.4f),
                  2);
    Plane back   (Vec3(0,0,-2),
                  Vec3(0,0,-1),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f),
                  2);

    std::vector<Plane*> planes;
    planes.push_back(&floor);
    planes.push_back(&ceiling);
    planes.push_back(&left);
    planes.push_back(&right);
    planes.push_back(&back);

    // SPHERES
    // Sphere s(position,radius,ambient,diffuse,specular,alpha)
    Sphere s1   (Vec3(0,0,0),
                 1,
                 Colour(.6f,.2f,.6f),
                 Colour(.6f,.2f,.6f),
                 Colour(.6f,.2f,.6f),
                 5);
    Sphere s2   (Vec3(2,2,1),
                 1,
                 Colour(.6f,.2f,.6f),
                 Colour(.6f,.2f,.6f),
                 Colour(.6f,.2f,.6f),
                 5);
    Sphere s3   (Vec3(-2,-.5f,-.5f),
                 .5f,
                 Colour(.6f,.6f,.6f),
                 Colour(.6f,.6f,.6f),
                 Colour(.6f,.6f,.6f),
                 5);

    std::vector<Sphere*> spheres;
    spheres.push_back(&s1);
    //spheres.push_back(&s2);
    //spheres.push_back(&s3);

    // TRIANGLES
    // Triangle t(v1, v2, v3, ambient, diffuse, specular, alpha)
    // normal can be any length and it will be normalized during construction
    Triangle t1(Vec3(0,-1,0),
                Vec3(1,1,0),
                Vec3(1,-1,0),
                Colour(.9f,.9f,.9f),
                Colour(.9f,.9f,.9f),
                Colour(.9f,.9f,.9f),
                2);
    Triangle t2(Vec3(-2,0,-1),
                Vec3(-1,1,-1),
                Vec3(0,0,0),
                Colour(.9f,.9f,.9f),
                Colour(.9f,.9f,.9f),
                Colour(.9f,.9f,.9f),
                2);

    std::vector<Triangle*> triangles;
    //triangles.push_back(&t1);
    //triangles.push_back(&t2);

    // DEFINE LIGHTING SOURCES
    // {position,ambient,diffuse,specular}
    LightSource L(Vec3(-1.9f,1.9f,1.9f),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f),
                  Colour(.6f,.6f,.6f));

    bool posXright = true;
    setup(cam,im,posXright,planes,spheres,triangles,L);

    // INITIALIZE LOOP VARIABLES
    Ray pixel;
    Objs objects = Objs(planes,spheres,triangles);
    Obj* closestObject = nullptr;
    float distance = 0.0f;

    for (int row = 0; row < im.image.rows(); ++row) {
        for (int col = 0; col < im.image.cols(); ++col) {

            pixel.constructPrimary(im,row,col,cam);
            // get both the object and the distance
            std::tie(closestObject,distance) = objects.findFirstObject(pixel);

            if (closestObject != nullptr) {
                im.image(row,col) = L.illuminate(objects,closestObject,pixel,pixel.at(distance),1);
            } // else the pixel stays black
        }
    }

    bmpwrite("../../out.bmp", im.image);
    imshow(im.image);

    return EXIT_SUCCESS;
}
