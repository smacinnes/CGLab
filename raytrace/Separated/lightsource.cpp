#include "lightsource.h"

LightSource::LightSource(const Vec3& pos,
                         const Colour& a,
                         const Colour& d,
                         const Colour& s,
                         int depth) :
    position(pos),
    ambient(a),
    diffuse(d),
    specular(s),
    maxDepth(depth){}

// bind a single pixel value in range [0.0f,1.0f]
void LightSource::bindPixel(float& value){
    if (value <= 0.0f) value = 0.0f;
    if (value >= 1.0f) value = 1.0f;
}

// bind a all pixel values in range [0.0f,1.0f]
void LightSource::bindPixelValues(Vec3& v){
    bindPixel(v(0));
    bindPixel(v(1));
    bindPixel(v(2));
}

void LightSource::bindValues(){
    bindPixelValues(Ia);
    // if diffuse <= 0, specular must be 0
    for(int i=0;i<3;i++){ if(Id(i)<0.0f){ Is(i)=0.0f;}}
    bindPixelValues(Id);
    bindPixelValues(Is);
}

Vec3 LightSource::mult(const Vec3& v1,const Vec3& v2){
    return Vec3(v1(0)*v2(0),v1(1)*v2(1),v1(2)*v2(2));
}

Colour LightSource::illuminate(Objs& objs,
                  const Obj* o,
                  const Ray& r,
                  const Vec3& intersection,
                  int recursionDepth){
    // ambient light calculation
    Ia = mult(o->ka,ambient);

    // create ray from intersection to light source
    // and store the distance between those points
    shadowLength = shadow.createBetween(intersection,position,0.01f);
    // check for objects along the shadow ray
    std::tie(obj,interLength) = objs.findFirstObject(shadow);
    // if there is an object before the light, don't calculate Id or Is
    //if (false){
    if (interLength < shadowLength){
        Id = Colour(0.0f, 0.0f, 0.0f);
        Is = Colour(0.0f, 0.0f, 0.0f);
    } else {
        // diffused light calculation
        normal = o->normal(intersection);
        lightAngle = (position-intersection).normalized();
        Id = mult(lightAngle.dot(normal)*o->kd,diffuse);

        // specular light calculation
        R = 2*lightAngle.dot(normal)*normal-lightAngle;
        Is = mult(o->ks*pow(R.dot(-r.dir),o->alpha),specular);

        // reflection calculations
        if (recursionDepth < maxDepth) {

        }

    }
    bindValues();
    return (Ia + Id + Is);
}
