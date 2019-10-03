#include "triangle.h"

Triangle::Triangle(const Vec3& va,
             const Vec3& vb,
             const Vec3& vc,
             const Vec3& a,
             const Vec3& d,
             const Vec3& s,
             float al) :
    Obj(a,d,s,al),
    v1(va),
    v2(vb),
    v3(vc) {
    edge1 = v2 - v1;
    edge2 = v3 - v1;
    norm = edge1.cross(edge2).normalized();
}

bool Triangle::equal(float a, float b, float e){
    return std::abs(a-b) <= e;
}

float Triangle::intersects(const Ray& r){
    h = r.dir.cross(edge2);
    a = edge1.dot(h);
    if (equal(a,0.0f,0.0001f)) return -1.0f;    // parallel
    f = 1.0f/a;
    s = r.orig - v1;
    u = f*s.dot(h);
    if (u < 0.0f || u > 1.0f) return -1.0f;     // outside triangle
    q = s.cross(edge1);
    v = f*r.dir.dot(q);
    if (v < 0.0f || u+v > 1.0f) return -1.0f;   // outside triangle
    t = f*edge2.dot(q);
    if (t > 0.0f) return t;                     // valid intersection
    return -1.0f;                               // triangle behind ray
}

Vec3 Triangle::normal(const Vec3&) const {
    return norm;
}
