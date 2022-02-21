//--------------------------------------------------------------------------------------
// Vector4 class (cut down version), to hold points and vectors for matrix work
//--------------------------------------------------------------------------------------
// Code in .cpp file

#pragma once

class CVector3;

class CVector4
{
	// Concrete class - public access
public:
	// Vector components
	float x;
	float y;
	float z;
	float w;

	/*-----------------------------------------------------------------------------------------
		Constructors
	-----------------------------------------------------------------------------------------*/

	// Default constructor - leaves values uninitialised (for performance)
	CVector4();

	// Construct with 4 values
	CVector4(float xIn,
	         float yIn,
	         float zIn,
	         float wIn);

	// Construct with CVector3 and a float for the w value (use to initialise with points (w=1) and vectors (w=0))
	CVector4(const CVector3& vIn,
	         float           wIn);

	// Construct using a pointer to 4 floats
	CVector4(const float* elts);

};

	CVector4 operator+(const CVector4& v1, const CVector4& v2);
