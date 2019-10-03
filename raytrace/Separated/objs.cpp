#include "objs.h"

Objs::Objs(const std::vector<Plane*>& p,
         const std::vector<Sphere*>& s,
         const std::vector<Triangle*>& t){
    for(auto it=p.begin();it!=p.end();++it){
        objects.push_back(*it);
    }
    for(auto it=s.begin();it!=s.end();++it){
        objects.push_back(*it);
    }
    for(auto it=t.begin();it!=t.end();++it){
        objects.push_back(*it);
    }
}

// return the closest object as well as the distance along the ray
std::tuple<Obj*,float> Objs::findFirstObject(const Ray& r){
    // reset variables from last call
    closestObj = nullptr;
    minDist = std::numeric_limits<float>::max();
    dist = -0.1f;

    // for every object in scene
    for(auto it=objects.begin();it!=objects.end();++it){

        // this will be t in x=o+td equation of line
        // each intersects() method will return -1 if there
        // is no VALID intersection (positive, not infinite etc)
        dist = (*it)->intersects(r);

        if (0.0f < dist && dist < minDist) {
            // then it is the closest object so far, so store
            // the dist and the reference to the object
            minDist = dist;
            closestObj = *it;
        }
    }
    // closestObj will still be nullptr if the ray didn't hit anything
    return std::make_tuple(closestObj,minDist);
}

