#ifndef LIGHTSOURCE_H
#define LIGHTSOURCE_H

#include "OpenGP/Image/Image.h"
#include "ray.h"
#include "obj.h"
#include "objs.h"
using namespace OpenGP;
using Colour = Vec3;

class LightSource {
public:
    Vec3 position;
    Colour ambient,diffuse,specular;
    int maxDepth;
private:
    Vec3 normal,lightAngle,R;
    Colour Ia,Id,Is;
    Ray shadow;
    Obj* obj;
    float shadowLength,interLength;
public:
    LightSource (const Vec3&,const Colour&,const Colour&,const Colour&,int);

    // bind a single pixel value in range [0.0f,1.0f]
    void bindPixel(float&);

    // bind a all pixel values in range [0.0f,1.0f]
    void bindPixelValues(Vec3&);

    // ensure all light levels fall in correct range
    void bindValues();

    // multiply two vectors elementwise (useful for colours)
    Vec3 mult(const Vec3&,const Vec3&);

    // determine the colour of a particular pixel
    // use Phong shading method
    // use shadow rays
    //
    Colour illuminate(Objs&,const Obj*,const Ray&,const Vec3&,int);
};

#endif // LIGHTSOURCE_H
