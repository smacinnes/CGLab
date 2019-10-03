#include "sphere.h"

Sphere::Sphere(const Vec3& c,
           float r,
           const Vec3& a,
           const Vec3& d,
           const Vec3& s,
           float al) :
    Obj(a,d,s,al),
    center(c),
    radius(r){}

float Sphere::intersects(const Ray& r){
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

Vec3 Sphere::normal(const Vec3& p) const {
    return (p-center).normalized();
}

