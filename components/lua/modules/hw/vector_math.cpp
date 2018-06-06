#include "vector_math.h"

SF3dVector::SF3dVector ( float _x, float _y, float _z )
{
	x = _x;
	y = _y;
	z = _z;
}

SF3dVector::SF3dVector ()
{
	x = y = z = 0.0;
}

float SF3dVector::getLength() const
{
	return (float)(sqrt(SQR(x)+SQR(y)+SQR(z)));
}

void SF3dVector::Normalize()
{
	float l = getLength();
	if (l == 0.0f) {
	   x = y = z = 0.0;
       return;
    }
	x /= l;
	y /= l;
	z /= l;
}

SF3dVector SF3dVector::operator+ (const SF3dVector &v)
{
	return SF3dVector(x+v.x, y+v.y, z+v.z);
}

SF3dVector SF3dVector::operator- (const SF3dVector &v)
{
	return SF3dVector(x-v.x, y-v.y, z-v.z);
}

SF3dVector SF3dVector::operator* (float r)
{
	return SF3dVector(x*r, y*r, z*r);
}

SF3dVector operator * ( const float r, const SF3dVector &v )
{
	return SF3dVector(v.x*r, v.y*r, v.z*r);
}

SF3dVector SF3dVector::operator/ (float r)
{
	return SF3dVector(x/r, y/r, z/r);
}

SF3dVector& SF3dVector::operator += (const SF3dVector &v)
{
    x+=v.x;
    y+=v.y;
    z+=v.z;
    return *this;
}

SF3dVector& SF3dVector::operator -= (const SF3dVector &v)
{
    x-=v.x;
    y-=v.y;
    z-=v.z;
    return *this;
}
SF3dVector& SF3dVector::operator *= (float r)
{
    x*=r;
    y*=r;
    z*=r;
    return *this;
}
SF3dVector& SF3dVector::operator /= (float r)
{
    x/=r;
    y/=r;
    z/=r; 
    return *this;
}
float SF3dVector::operator* (const SF3dVector &v)	//dot product
{
	return x*v.x + y*v.y + z*v.z;
}
SF3dVector CrossProduct (const SF3dVector &u, const SF3dVector &v)
{
	return SF3dVector(u.y*v.z - u.z*v.y, 
                      u.z*v.x - u.x*v.z,
                      u.x*v.y - u.y*v.x);
}

SF3dMatrix::SF3dMatrix (
	float _x1, float _y1, float _z1,
	float _x2, float _y2, float _z2,
	float _x3, float _y3, float _z3
)
{
	x1 = _x1;
	y1 = _y1;
	z1 = _z1;
	x2 = _x2;
	y2 = _y2;
	z2 = _z2;
	x3 = _x3;
	y3 = _y3;
	z3 = _z3;
}

SF3dMatrix::SF3dMatrix ()
{
	x1 = y1 = z1 = x2 = y2 = z2 = x3 = y3 = z3 = 0.0;
}

SF3dVector SF3dMatrix::operator * (const SF3dVector &o) {
	SF3dVector ret;

	ret.x = x1*o.x + y1*o.y + z1*o.z;
	ret.y = x2*o.x + y2*o.y + z2*o.z;
	ret.z = x3*o.x + y3*o.y + z3*o.z;

	return ret;
}
