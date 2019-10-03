#ifndef IMAGEPLANE_H
#define IMAGEPLANE_H

#include "OpenGP/Image/Image.h"
#include "camera.h"
using namespace OpenGP;

using Colour = Vec3;

// Class defining all properties of the image plane
class ImagePlane {
private:
    int wResolution,hResolution;         // default is 640x480
    float viewingAngle,distToCam,hwRatio,halfWidth;
    Vec3 center;
public:
    Image<Colour> image;
    Vec3 llc,pixRi,pixUp;

    // constructor for wRes,hRes,viewingAngle and distToCam
    ImagePlane(int,int,float,float);

    void setup(const Camera&);
};

#endif // IMAGEPLANE_H
