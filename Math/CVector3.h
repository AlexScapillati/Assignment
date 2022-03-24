//--------------------------------------------------------------------------------------
// Vector3 class (cut down version), to hold points and vectors
//--------------------------------------------------------------------------------------
// Code in .cpp file

#ifndef _CVECTOR3_H_DEFINED_
#define _CVECTOR3_H_DEFINED_
#include "CVector4.h"

class CVector3
{
	// Concrete class - public access
public:
	// Vector components
	float x;
	float y;
	float z;

	/*-----------------------------------------------------------------------------------------
		Constructors
	-----------------------------------------------------------------------------------------*/

	// Default constructor - leaves values uninitialised (for performance)
	CVector3() {}

	// Construct with 3 values
	CVector3(const float xIn, const float yIn, const float zIn)
	{
		x = xIn;
		y = yIn;
		z = zIn;
	}

	// Construct using a pointer to three floats
	CVector3(const float* pfElts)
	{
		x = pfElts[0];
		y = pfElts[1];
		z = pfElts[2];
	}

	CVector3(const CVector4 in)
	{
		x = in.x;
		y = in.y;
		z = in.z;
	}

	float* GetValuesArray()
	{
		return &x;
	}

	/*-----------------------------------------------------------------------------------------
		Member functions
	-----------------------------------------------------------------------------------------*/

	// Addition of another vector to this one, e.g. Position += Velocity
	CVector3& operator+= (const CVector3& v);

	// Subtraction of another vector from this one, e.g. Velocity -= Gravity
	CVector3& operator-= (const CVector3& v);

	// Negate this vector (e.g. Velocity = -Velocity)
	CVector3& operator- ();

	// Plus sign in front of vector - called unary positive and usually does nothing. Included for completeness (e.g. Velocity = +Velocity)
	CVector3& operator+ ();

	CVector3& operator%=(float s);
	CVector3& operator/=(float v);

	// Multiply vector by scalar (scales vector);
	CVector3& operator*= (float s);

	static CVector3 Rand();

	float operator[](int index) const;

	float Magnitude() const;
};

/*-----------------------------------------------------------------------------------------
	Non-member operators
-----------------------------------------------------------------------------------------*/

// Vector-vector addition
CVector3 operator+ (const CVector3& v, const CVector3& w);

// Vector-vector subtraction
CVector3 operator- (const CVector3& v, const CVector3& w);

// Vector-scalar subtraction
CVector3 operator- (const CVector3& v, const float& w);
CVector3 operator- (const float& w,const CVector3& v);

// Vector-scalar subtraction
CVector3 operator+ (const CVector3& v, const float& w);
CVector3 operator+ (const float& w,const CVector3& v);

// Vector-scalar multiplication
CVector3 operator* (const CVector3& v, float s);
CVector3 operator* (float s, const CVector3& v);

// Vector-scalar division
CVector3 operator/ (const CVector3& v, float s);
CVector3 operator/ (float s,const CVector3& v);

// Vector-vector multiplication
CVector3 operator* (const CVector3& v, const CVector3& w);


/*-----------------------------------------------------------------------------------------
	Non-member functions
-----------------------------------------------------------------------------------------*/

// Dot product of two given vectors (order not important) - non-member version
float Dot(const CVector3& v1, const CVector3& v2);

// Cross product of two given vectors (order is important) - non-member version
CVector3 Cross(const CVector3& v1, const CVector3& v2);

// Return unit length vector in the same direction as given one
CVector3 Normalise(const CVector3& v);

// Returns length of a vector
float Length(const CVector3& v);

CVector3 ToDegrees(CVector3 v);

CVector3 ToRadians(CVector3 v);

CVector3 ScaleBetween(CVector3 v, float minAllowed, float maxAllowed, CVector3 min, CVector3 max);

CVector3 Reflect(const CVector3& vec, const CVector3& surfaceNormal);

#endif // _CVECTOR3_H_DEFINED_
