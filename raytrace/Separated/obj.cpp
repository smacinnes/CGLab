#include "obj.h"

Obj::Obj(const Vec3& a,
        const Vec3& d,
        const Vec3& s,
        float al) :
        ka(a),
        kd(d),
        ks(s),
        alpha(al) {}

