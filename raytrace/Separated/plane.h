#ifndef PLANE_H
#define PLANE_H

#include "OpenGP/Image/Image.h"
#include "obj.h"
using namespace OpenGP;

// defines an infinite plane (ex the floor)
// use triangles (yet to be implemented) for walls
class Plane : public Obj{
public:
    Vec3 point,norm;
private:
    float temp;
public:
    Plane(const Vec3&,const Vec3&,const Vec3&,const Vec3&,const Vec3&,float);

    // if two floats are close enough
    bool equal(float, float, float);

    /* returns the distance along the ray from the plane to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning a single positive solution
     */
    float intersects(const Ray& r);

    // return normal (parameter there for inheritance)
    Vec3 normal(const Vec3&) const;
};

#endif // PLANE_H
