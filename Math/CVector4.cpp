//--------------------------------------------------------------------------------------
// Vector4 class (cut down version), to hold points and vectors
//--------------------------------------------------------------------------------------

#include "CVector4.h"

#include "CVector3.h"

 CVector4::CVector4()
{}

 CVector4::CVector4(const float xIn,
	const float yIn,
	const float zIn,
	const float wIn)
{
	x = xIn;
	y = yIn;
	z = zIn;
	w = wIn;
}

 CVector4::CVector4(const CVector3& vIn,
	const float wIn)
{
	x = vIn.x;
	y = vIn.y;
	z = vIn.z;
	w = wIn;
}

 CVector4::CVector4(const float* elts)
{
	x = elts[0];
	y = elts[1];
	z = elts[2];
	w = elts[3];
}

 CVector4 operator+(const CVector4& v, const CVector4& w)
 {
	 return CVector4{ v.x + w.x, v.y + w.y, v.z + w.z, v.w + w.w};
 }
 
