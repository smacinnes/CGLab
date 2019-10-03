#include "imageplane.h"

ImagePlane::ImagePlane(int wRes,int hRes,float angle,float dist):
    wResolution(wRes),
    hResolution(hRes),
    viewingAngle(angle),
    distToCam(dist) {
    image  = Image<Colour>(hResolution, wResolution);
    hwRatio = float(hResolution)/wResolution;
}

void ImagePlane::setup(const Camera& c){
    center = c.position+c.viewingDir*distToCam;
    halfWidth = distToCam*tanf(viewingAngle*float(M_PI)/360.0f);
    llc = center - c.rightAxis*halfWidth - c.upAxis*halfWidth*hwRatio;
    pixRi = 2*halfWidth/(wResolution-1)*c.rightAxis;
    pixUp = 2*halfWidth*hwRatio/(hResolution-1)*c.upAxis;
}

