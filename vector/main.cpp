#include<iostream>
#include "vector.h"

using namespace std;

int main(int, char**){

    cout<<"3D Vector Math Example"<<endl;

    Vector vec1; //vec1 is an object of Vector class
    Vector vec2=Vector(5.0f, 0.0f, 0.0f); //Constructorfor initializing x, y, z of vec2
    vec1=Vector();

    cout<<"Input vectors:"<<endl;
    cout<<"vec1: "<<vec1; //You should see some garbage values for vec1.x, vec1.y, vec1.z
    cout<<"vec2: "<<vec2;
    cout<<endl;

    //set vec1 to a zero vector
    vec1.Zero();

    cout<<"Assign vec1 to zero vector:"<<endl;
    cout<<vec1<<endl;

    //vector addition
    cout<<"Sum of vec1 and vec2:"<<endl;
    cout<<vec1+vec2<<endl;

    //vector subtraction
    cout<<"Subtract vec1 from vec2:"<<endl;
    cout<<vec2-vec1<<endl;


    //scalar multiplication
    cout<<"Multipl vec2 with 2:"<<endl;
    int s=2;
    cout<<vec2*s<<endl;

    //assignment- set vec1 to vec2
    cout<<"Assign vec1 with 2*vec2:"<<endl;
    vec1=vec2*s;
    cout<<vec1<<endl;


    //dot product
    cout<<"Now"<<endl;
    cout<<"vec1:"<<vec1;
    cout<<"vec2:"<<vec2;
    cout<<"vec1 dot vec2:"<<endl;
    cout<<vec1*vec2<<endl;
    cout<<endl;

    //Vector magnitude
    Vector vec3=Vector(2.0f, 5.0f, 2.0f);
    cout<<"Euclidean norm of vector " <<vec3;
    cout<<vec3.VectorMag()<<endl;
    cout<<endl;

    //cross product
    cout<<"vec2:"<<vec2;
    cout<<"vec3:"<<vec3;
    cout<<"vec2 cross vec3"<<endl;
    cout<<vec2.CrossProduct(vec3)<<endl;



    return EXIT_SUCCESS;
}

