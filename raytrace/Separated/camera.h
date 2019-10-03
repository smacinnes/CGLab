#ifndef CAMERA_H
#define CAMERA_H

#include "OpenGP/Image/Image.h"
using namespace OpenGP;

// Class defining the position and orientation of the camera
// TODO: add easier adjustment of tilting function (ex auto calculate up
// vector from the tilt in radians)
class Camera {
public:
    Vec3 position,lookingAt,up;        // up = (0,1,0) is no tilt
    Vec3 viewingDir,rightAxis,upAxis;

    Camera(const Vec3&,const Vec3&,const Vec3&);

    void setup();
};

#endif // CAMERA_H
