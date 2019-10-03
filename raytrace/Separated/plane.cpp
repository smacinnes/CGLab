#include "plane.h"

Plane::Plane(const Vec3& p,
          const Vec3& n,
          const Vec3& a,
          const Vec3& d,
          const Vec3& s,
          float al) :
    Obj(a,d,s,al),
    point(p),
    norm(n.normalized()){}

bool Plane::equal(float a, float b, float e){
    return std::abs(a-b) <= e;
}

float Plane::intersects(const Ray& r){
    temp = norm.dot(r.dir);                     // hold dot prod op
    if(equal(temp,0.0f,0.0001f)) return -1.0f;  // if dot~=0
    temp = (point-r.orig).dot(norm)/temp;       // hold scalar solution
    if (temp > 0) return temp;                  // ensure (+) solution
    return -1.0f;
}

Vec3 Plane::normal(const Vec3&) const {
    return norm;
}

