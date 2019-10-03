#ifndef SPHERE_H
#define SPHERE_H

#include "OpenGP/Image/Image.h"
#include "obj.h"
using namespace OpenGP;

// defines a sphere via a point and a radius
class Sphere : public Obj {
public:
    Vec3 center;
    float radius;
private:
    Vec3 co;      // center of sphere to origin of ray
    float disc,t1,t2,b;
public:
    Sphere(const Vec3&,float,const Vec3&,const Vec3&,const Vec3&,float);

    /* returns the distance along the ray from the sphere to the
     * ray's origin if there is a valid intersection and -1 otherwise
     * "valid" meaning that at least one solution is positive
     * two pos:    ray starts before sphere    (return smallest solution)
     * one pos:    ray starts inside sphere    (cross section - optional)
     * two neg:    ray starts after sphere     (ignore)
     * no real:    ray does not intersect      (ignore)
     */
    float intersects(const Ray&);

    // return normal
    Vec3 normal(const Vec3& p) const;
};

#endif // SPHERE_H
