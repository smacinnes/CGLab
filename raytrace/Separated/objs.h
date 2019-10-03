#ifndef OBJS_H
#define OBJS_H

#include "OpenGP/Image/Image.h"
#include "obj.h"
#include "plane.h"
#include "sphere.h"
#include "triangle.h"
using namespace OpenGP;

class Objs {
public:
    std::vector<Obj*> objects;
private:
    Obj* closestObj = nullptr;
    float minDist = std::numeric_limits<float>::max();
    float dist = -1.0f;
public:
    Objs(const std::vector<Plane*>&,
         const std::vector<Sphere*>&,
         const std::vector<Triangle*>&);

    // return the closest object as well as the distance along the ray
    std::tuple<Obj*,float> findFirstObject(const Ray&);
};

#endif // OBJS_H
