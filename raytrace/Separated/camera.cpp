#include "camera.h"

Camera::Camera(const Vec3& pos,const Vec3& look,const Vec3& u) :
        position(pos),
        lookingAt(look),
        up(u){}

void Camera::setup(){
    viewingDir = (lookingAt - position).normalized();
    rightAxis = (up.cross(viewingDir)).normalized();
    upAxis = viewingDir.cross(rightAxis); // will be unit vec
}
