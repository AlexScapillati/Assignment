//--------------------------------------------------------------------------------------
// Matrix4x4 class (cut down version) to hold matrices for 3D
//--------------------------------------------------------------------------------------
// Code in .cpp file

#ifndef _CMATRIX4X4_H_DEFINED_
#define _CMATRIX4X4_H_DEFINED_

#include "CVector3.h"
#include "CVector4.h"
#include "MathHelpers.h"

// Matrix class
class CMatrix4x4
{
	// Concrete class - public access
public:
	// Matrix elements
	float e00, e01, e02, e03;
	float e10, e11, e12, e13;
	float e20, e21, e22, e23;
	float e30, e31, e32, e33;

	/*-----------------------------------------------------------------------------------------
		Member functions
	-----------------------------------------------------------------------------------------*/

	// Set a single row (range 0-3) of the matrix using a CVector3. Fourth element left unchanged
	// Can be used to set position or x,y,z axes in a matrix
	void SetRow(int iRow, const CVector3& v);

	// Get a single row (range 0-3) of the matrix into a CVector3. Fourth element is ignored
	// Can be used to access position or x,y,z axes from a matrix
	CVector3 GetRow(int iRow);

	// Initialise this matrix with a pointer to 16 floats
	void SetValues(float* matrixValues) { *this = *reinterpret_cast<CMatrix4x4*>(matrixValues); }

	// Helper functions
	CVector3 GetXAxis() { return GetRow(0); }
	CVector3 GetYAxis() { return GetRow(1); }
	CVector3 GetZAxis() { return GetRow(2); }
	CVector3 GetPosition() { return GetRow(3); }
	CVector3 GetEulerAngles() const;
	CVector3 GetScale() { return { Length(GetXAxis()), Length(GetYAxis()) , Length(GetZAxis()) }; }

	
	// Rotate an affine transformation by given angle (radians) around world X axis & local origin
	// Note: The position (translation) will not be altered, use RotateWorldX to perform a world X
	// rotation around world origin
	void RotateX( const float x )
	{
		// Perform minimum of calculations rather than use full matrix multiply
		float sX, cX;
		SinCos( x, &sX, &cX );
		float t;
		t   = e01*sX + e02*cX;
		e01 = e01*cX - e02*sX;
		e02 = t;
		t   = e11*sX + e12*cX;
		e11 = e11*cX - e12*sX;
		e12 = t;
		t   = e21*sX + e22*cX;
		e21 = e21*cX - e22*sX;
		e22 = t;
	}

	// Rotate an affine transformation by given angle (radians) around world Y axis & local origin
	// Note: The position (translation) will not be altered, use RotateWorldY to perform a world Y
	// rotation around world origin
	void RotateY( const float y )
	{
		// Perform minimum of calculations rather than use full matrix multiply
		float sY, cY;
		SinCos( y, &sY, &cY );
		float t;
		t   = e00*cY + e02*sY;
		e02 = e02*cY - e00*sY;
		e00 = t;
		t   = e10*cY + e12*sY;
		e12 = e12*cY - e10*sY;
		e10 = t;
		t   = e20*cY + e22*sY;
		e22 = e22*cY - e20*sY;
		e20 = t;
	}

	// Rotate an affine transformation by given angle (radians) around world Z axis & local origin
	// Note: The position (translation) will not be altered, use RotateWorldZ to perform a world Z
	// rotation around world origin
	void RotateZ( const float z )
	{
		// Perform minimum of calculations rather than use full matrix multiply
		float sZ, cZ;
		SinCos( z, &sZ, &cZ );
		float t;
		t   = e00*sZ + e01*cZ;
		e00 = e00*cZ - e01*sZ;
		e01 = t;
		t   = e10*sZ + e11*cZ;
		e10 = e10*cZ - e11*sZ;
		e11 = t;
		t   = e20*sZ + e21*cZ;
		e20 = e20*cZ - e21*sZ;
		e21 = t;
	}


	float* GetArray() {
		return &e00;
	}

	// Post-multiply this matrix by the given one
	CMatrix4x4& operator*=(const CMatrix4x4& m);

	CVector4 operator*=(const CVector4& v) const;

	// Make this matrix an affine 3D transformation matrix to face from current position to given
	// target (in the Z direction). Can pass up vector for the constructed matrix and specify
	// handedness (right-handed Z axis will face away from target)
	// Will retain the matrix's current scaling
	void FaceTarget(const CVector3& target);

	// Transpose the matrix (rows become columns). There are two ways to store a matrix, by rows or by columns.
	// Different apps use different methods. Use Transpose to swap when necessary.
	void Transpose();
	CMatrix4x4 RotationMatrix();
};

/*-----------------------------------------------------------------------------------------
	Non Member Operators
-----------------------------------------------------------------------------------------*/

// Matrix-matrix multiplication
CMatrix4x4 operator*(const CMatrix4x4& m1, const CMatrix4x4& m2);

// Return the given CVector4 transformed by the given matrix
CVector4 operator*(const CVector4& v, const CMatrix4x4& m);

/*-----------------------------------------------------------------------------------------
  Non-member functions
-----------------------------------------------------------------------------------------*/

// The following functions create a new matrix holding a particular transformation
// They can be used as temporaries in calculations, e.g.
//     CMatrix4x4 m = MatrixScaling( 3.0f ) * MatrixTranslation( CVector3(10.0f, -10.0f, 20.0f) );

// Return an identity matrix
CMatrix4x4 MatrixIdentity();

// Return a translation matrix of the given vector
CMatrix4x4 MatrixTranslation(const CVector3& t);

// Return an X-axis rotation matrix of the given angle (in radians)
CMatrix4x4 MatrixRotationX(float x);

// Return a Y-axis rotation matrix of the given angle (in radians)
CMatrix4x4 MatrixRotationY(float y);

// Return a Z-axis rotation matrix of the given angle (in radians)
CMatrix4x4 MatrixRotationZ(float z);

// Return a matrix that is a scaling in X,Y and Z of the values in the given vector
CMatrix4x4 MatrixScaling(const CVector3& s);

// Return a matrix that is a uniform scaling of the given amount
CMatrix4x4 MatrixScaling(const float s);

// Return the inverse of given matrix assuming that it is an affine matrix
// Advanced calulation needed to get the view matrix from the camera's positioning matrix
CMatrix4x4 InverseAffine(const CMatrix4x4& m);

#endif // _CMATRIX4X4_H_DEFINED_
