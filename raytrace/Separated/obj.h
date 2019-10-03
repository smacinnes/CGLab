#ifndef OBJ_H
#define OBJ_H

#include "OpenGP/Image/Image.h"
#include "ray.h"
using namespace OpenGP;

// All objects in the scene will derive from this class and
// have these material properties and methods for simple design
class Obj {
public:
    Colour ka,kd,ks;
    float alpha;

    Obj(const Vec3&,const Vec3&,const Vec3&,float);

    // returns the dist to the valid intersection if there is one, -1 otherwise
    // ray is o+td, this returns t or -1
    virtual float intersects(const Ray&) = 0;

    // returns the normal at the specified point
    virtual Vec3 normal(const Vec3&) const = 0;

    // virtual destructor - compiler complains without it
    virtual ~Obj(){}
};

#endif // OBJ_H
