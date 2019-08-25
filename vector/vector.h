#ifndef VECTOR_H
#define VECTOR_H

#include <cmath>
#include<iostream>


class Vector
{
public:
    //constructors (overlaoded constructors-3 constructors taking different types of arguments)
    Vector(){}

    Vector(float nx, float ny, float nz):x(nx), y(ny), z(nz){}

    Vector(const Vector &v): x(v.x), y(v.y), z(v.z){}

    //Assignment (operator overlaoding, the operstor '=' is used to asssign one vector to another)
    Vector &operator =(const Vector &);
    void Zero(); //Set to zero vector

    //Equality checking
    bool operator ==(const Vector &) const;

    //Vector operations (operator overlaoding examples)

    Vector operator -() const; //Negation
    Vector operator +(const Vector &) const; //vector addition
    Vector operator -(const Vector &) const; //vector subtraction
    Vector operator *(float) const; //multiplication with scalar
    Vector operator /(float) const; //division by scalar

    //Normalize a vector
    void Normalize();

    //dot product
    float operator *(const Vector &) const;//use '*' operator for dot product

    //cross product
    Vector CrossProduct(const Vector &);

    //Magnitude
    float VectorMag();

    //output the vector
    friend std::ostream& operator<< (std::ostream&, const Vector&); //friend class

public:
    float x, y, z;
};

#endif // VECTOR_H
