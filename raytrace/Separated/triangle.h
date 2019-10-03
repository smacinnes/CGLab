#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "OpenGP/Image/Image.h"
#include "obj.h"
using namespace OpenGP;

// define a triangle using 3 points
class Triangle : public Obj {
public:
    Vec3 v1,v2,v3,edge1,edge2,norm;
private:
    Vec3 h,s,q;
    float a,f,u,v,t;
public:
    Triangle(const Vec3&,const Vec3&,const Vec3&,const Vec3&,const Vec3&,const Vec3&,float);

    // if two floats are close enough
    bool equal(float a, float b, float e);

    /* returns the distance along the ray from the triangle to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning a single positive solution inside the triangle
     * using the Moller-Trumbore algorithm
     */
    float intersects(const Ray&);

    // return normal (need param for overloading)
    Vec3 normal(const Vec3&) const;
};

#endif // TRIANGLE_H
