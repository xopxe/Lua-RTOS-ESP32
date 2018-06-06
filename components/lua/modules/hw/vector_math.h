#ifndef __VECTOR_MATH_H_
#define __VECTOR_MATH_H_

#include "math.h"

#define PI 3.1415926535897932384626433832795
#define PIdiv180 (PI/180.0)
#define SQR(x) (x*x)
#define SGN(x) (x<0?-1:1)
#define ABS(x) (x<0?-x:x)
#define NULL_VECTOR F3dVector(0.0f,0.0f,0.0f)


/////////////////////////////////
//Note: All angles in degrees  //
/////////////////////////////////

class SF3dVector  //Float 3d-vect, normally used
{
    public:  
	float x,y,z;

    SF3dVector ( float x, float y, float z);
    SF3dVector ();
    ~SF3dVector () {}
    float getLength() const;
    void Normalize();

    SF3dVector operator+ (const SF3dVector &v);
    SF3dVector operator- (const SF3dVector &v);
    SF3dVector operator* (float r);
    friend SF3dVector operator * (const float scalar, const SF3dVector &other);
    SF3dVector operator/ (float r);
    SF3dVector& operator += (const SF3dVector &v);
    SF3dVector& operator -= (const SF3dVector &v);
    SF3dVector& operator *= (float r);
    SF3dVector& operator /= (float r);
    float operator* (const SF3dVector &v);	//dot product
};

SF3dVector CrossProduct (const SF3dVector &u, const SF3dVector &v);

class SF3dMatrix  //Float 3d-vect, normally used
{
    public:
	float x1,y1,z1,x2,y2,z2,x3,y3,z3;

	SF3dMatrix (
		float x1, float y1, float z1,
		float x2, float y2, float z2,
		float x3, float y3, float z3
	);
	SF3dMatrix ();
    ~SF3dMatrix () {}

    SF3dVector operator * (const SF3dVector &other);
};


class SF2dVector
{
    public:  
	float x,y;
    SF2dVector () { x=y=0.0; }
    SF2dVector (float _x, float _y) { x=_x; y=_y; }
    ~SF2dVector () {}
};


#endif //__VECTOR_MATH_H_
