#include "OpenGP/Image/Image.h"
#include "bmpwrite.h"

using namespace OpenGP;
using Colour = Vec3;

Colour red() { return Colour(255, 0, 0); }
Colour green() { return Colour(0, 255, 0); }
Colour blue() { return Colour(0, 0, 255); }

struct Triangle {
    Vec3 v1, v2, v3;
};

struct Line {
    Vec3 v1, v2;
};

float triangleArea(Vec3 v1, Vec3 v2, Vec3 v3) {
    //@TODO compute the triangle area
    return 0;
}

void rasterize(Triangle t, Image<Colour> &image, Colour color) {

    // Vectors projected onto z=0 plane
    Vec3 s1 = Vec3(t.v1(0), t.v1(1), 0);
    Vec3 s2 = Vec3(t.v2(0), t.v2(1), 0);
    Vec3 s3 = Vec3(t.v3(0), t.v3(1), 0);

    /// OPTIONAL: Restrict these bounds to only rasterize triangle bounding box
    int iMin = 0;
    int iMax = image.cols();
    int jMin = 0;
    int jMax = image.rows();

    float total_area = triangleArea(s1, s2, s3);

    for (int i = iMin;i < iMax;i++) {
        for (int j = jMin;j < jMax;j++) {

            // Location of fragment in image space
            Vec3 pt = Vec3(i, j, 0.0);
            

            /// TODO: Calculate barycentric coordinates of the fragment within current triangle
            float alpha = 0;
            float beta = 0;
            float gamma = 0;

            if (false /* TODO check if fragment is inside triangle **/)
            {
                image(j, i) = color;
            }


        }
    }

}

float LineEquation(const Vec3 & p0, const Vec3 & p1, float x, float y)
{
	float result = 0;
    //@TODO compute the implicit line equation -- four terms
    //(y0 - y1) x + (x1 - x0) y + x0y1 -x1y0
       
    return result;

}

void ImageCordBound(int & xCord, int & yCord, int cols, int rows)
{
    if (xCord < 0) xCord = 0;
    if (xCord >= cols) xCord = cols - 1;
    if (yCord < 0) yCord = 0;
    if (yCord >= rows) yCord = rows -1;
}
void rasterize(Line l, Image<Colour> &image) {

    // Vectors projected onto z=0 plane
    Vec3 s1 = Vec3(l.v1(0), l.v1(1), 0);
    Vec3 s2 = Vec3(l.v2(0), l.v2(1), 0);

    /// TODO: Calculate x-coordinates difference (deltax) and y-coordinates difference (deltay)
    float deltax = 0;
    float deltay = 0;

    if (deltax > deltay)
    {
        //going the x-direction
        //pre-processing the start and end points
        if (s1[0] > s2[0])
        {
            //swap
            Vec3 temp = s1;
            s1 = s2;
            s2 = temp;
        }
        int pixelStartX = (int)(s1[0]);
        int pixelStartY = (int)(s1[1]);
        ImageCordBound(pixelStartX,pixelStartY, image.cols(), image.rows());

        int pixelEndX = (int)(s2[0]);
        int pixelEndY = (int)(s2[1]);
        ImageCordBound(pixelEndX,pixelEndY, image.cols(), image.rows());


        ///@TODO the midpoint algorithm
        
    }
    else{
        //going the y-direction
        //pre-processing the start and end points
        if (s1[1] > s2[1])
        {
            //swap
            Vec3 temp = s1;
            s1 = s2;
            s2 = temp;
        }
        int pixelStartX = (int)(s1[0]);
        int pixelStartY = (int)(s1[1]);
        ImageCordBound(pixelStartX,pixelStartY, image.cols(), image.rows());

        int pixelEndX = (int)(s2[0]);
        int pixelEndY = (int)(s2[1]);
        ImageCordBound(pixelEndX,pixelEndY, image.cols(), image.rows());

        //@TODO the midpoint algorithm
        
    }


}

int main(int, char**){

    int wResolution = 500;
    int hResolution = 500;

    Image<Colour> image(wResolution, hResolution);

    //triangles
    Triangle t1 = {
        Vec3(100,100,0), Vec3(250,300,0), Vec3(400,100,0)
    };

    Triangle t2 = {
        Vec3(0,100,0), Vec3(100,200,0), Vec3(200,100,0)
    };

    Triangle t3 = {
        Vec3(500,100,0), Vec3(400,200,0), Vec3(300,100,0)
    };

    Triangle t4 = {
        Vec3(0,100,0), Vec3(0,0,0), Vec3(500,100,0)
    };

    Triangle t5 = {
        Vec3(0,0,0), Vec3(500,0,0), Vec3(500,100,0)
    };

    //Line
    Line l = {
        Vec3(0, 0, 0), Vec3(100, 100, 0)
    };
	
	//@TODO render rain in the sub region [(0,100), (500,100), (500,500), (0,500))]

    

    //@TODO render the sea (light blue) & mountains (green triangles)
    


    bmpwrite("../../out.bmp", image);
    imshow(image);

    return EXIT_SUCCESS;
}
