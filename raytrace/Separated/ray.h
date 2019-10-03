#ifndef RAY_H
#define RAY_H

#include "OpenGP/Image/Image.h"
#include "camera.h"
#include "imageplane.h"
using namespace OpenGP;

// the intersect() and related methods are stored in the object class definition
// for purposes of overloading
class Ray {
public:
    Vec3 orig,dir;

    // create a Ray between these two points
    // offset is distance along ray to move the start position
    // to avoid self intersection
    // return the distance between the points
    float createBetween(const Vec3&, const Vec3&, float);

    // intersect image plane at specified pixel
    void constructPrimary(const ImagePlane&,int,int,const Camera&);

    // ray location with given parameter
    Vec3 at(float) const;
};

#endif // RAY_H
