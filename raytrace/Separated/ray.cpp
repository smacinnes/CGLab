#include "ray.h"
#include "OpenGP/Image/Image.h"
#include "imageplane.h"
#include "camera.h"
using namespace OpenGP;

float Ray::createBetween(const Vec3& from, const Vec3& to, float offset){
    dir = (to-from).normalized();
    orig = from+offset*dir;
    return (to-orig).norm();
}

void Ray::constructPrimary(const ImagePlane& im,int row,int col,const Camera& cam){
    orig = cam.position;
    dir = (im.llc+im.pixRi*float(col-1)+im.pixUp*float(row-1)
           -cam.position).normalized();
    }

Vec3 Ray::at(float t) const {
    return orig+t*dir;
}
