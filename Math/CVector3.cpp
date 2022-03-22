//--------------------------------------------------------------------------------------
// Vector3 class (cut down version), to hold points and vectors
//--------------------------------------------------------------------------------------

#include "CVector3.h"

#include "MathHelpers.h"

/*-----------------------------------------------------------------------------------------
	Operators
-----------------------------------------------------------------------------------------*/

// Addition of another vector to this one, e.g. Position += Velocity
CVector3& CVector3::operator+= (const CVector3& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

// Subtraction of another vector from this one, e.g. Velocity -= Gravity
CVector3& CVector3::operator-= (const CVector3& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

// Negate this vector (e.g. Velocity = -Velocity)
CVector3& CVector3::operator- ()
{
	x = -x;
	y = -y;
	z = -z;
	return *this;
}

// Plus sign in front of vector - called unary positive and usually does nothing. Included for completeness (e.g. Velocity = +Velocity)
CVector3& CVector3::operator+ ()
{
	return *this;
}

// Multiply vector by scalar (scales vector);
CVector3& CVector3::operator*= (const float s)
{
	x *= s;
	y *= s;
	z *= s;
	return *this;
}

//Direct access with []
float CVector3::operator[](const int index) const
{
	if (index == 0)
	{
		return x;
	}
	if (index == 1)
	{
		return y;
	}
	if (index == 2)
	{
		return z;
	}
	return NULL;
}

CVector3& CVector3::Rand()
{
	return CVector3(Random(-1.0f, 1.0f), Random(-1.0f, 1.0f), Random(-1.0f,1.0f));
}

// Vector-vector addition
CVector3 operator+ (const CVector3& v, const CVector3& w)
{
	return CVector3{ v.x + w.x, v.y + w.y, v.z + w.z };
}

// Vector-vector subtraction
CVector3 operator- (const CVector3& v, const CVector3& w)
{
	return CVector3{ v.x - w.x, v.y - w.y, v.z - w.z };
}

CVector3 operator-(const CVector3& v, const float& w)
{
	return CVector3{ v.x - w, v.y -w,v.z -w};
}

CVector3 operator-(const float& w, const CVector3& v)
{
	return CVector3{ v.x - w, v.y -w,v.z -w};
}

CVector3 operator+(const CVector3& v, const float& w)
{
	return CVector3{ v.x + w, v.y + w,v.z + w};
}

CVector3 operator+(const float& w, const CVector3& v)
{
	return CVector3{ v.x + w, v.y + w,v.z + w};
}

// Vector-scalar multiplication
CVector3 operator* (const CVector3& v, float s)
{
	return CVector3{ v.x * s, v.y * s, v.z * s };
}

CVector3 operator* (float s, const CVector3& v)
{
	return CVector3{ v.x * s, v.y * s, v.z * s };
}

CVector3 operator/(const CVector3& v, float s)
{
	return CVector3{v.x/s,v.y/s,v.z/s};
}

CVector3 operator/(float s, const CVector3& v)
{
	return CVector3{v.x/s,v.y/s,v.z/s};
}

CVector3 operator*(const CVector3& v, const CVector3& w)
{
	return CVector3(v.x*w.x,v.y*w.y,v.z*w.z);
}

// Addition of another vector to this one, e.g. Position += Velocity
CVector3& CVector3::operator /= (const float v)
{
	x /= v;
	y /= v;
	z /= v;
	return *this;
}
/*-----------------------------------------------------------------------------------------
	Non-member functions
-----------------------------------------------------------------------------------------*/

// Dot product of two given vectors (order not important) - non-member version
float Dot(const CVector3& v1, const CVector3& v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// Cross product of two given vectors (order is important) - non-member version
CVector3 Cross(const CVector3& v1, const CVector3& v2)
{
	return CVector3{ v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x };
}

// Return unit length vector in the same direction as given one
CVector3 Normalise(const CVector3& v)
{
	const auto lengthSq = v.x * v.x + v.y * v.y + v.z * v.z;

	// Ensure vector is not zero length (use BaseMath.h float approx. fn with default epsilon)
	if (IsZero(lengthSq))
	{
		return CVector3{ 0.0f, 0.0f, 0.0f };
	}
	else
	{
		const auto invLength = Q_rsqrt(lengthSq);
		return CVector3{ v.x * invLength, v.y * invLength, v.z * invLength };
	}
}

// Returns length of a vector
float Length(const CVector3& v)
{
	return sqrt(Dot(v, v));
}

CVector3 ToDegrees(CVector3 v)
{
	return CVector3(ToDegrees(v.x), ToDegrees(v.y), ToDegrees(v.z));
}

CVector3 ToRadians(CVector3 v)
{
	return CVector3(ToRadians(v.x), ToRadians(v.y), ToRadians(v.z));
}

// Scale between two values (min allowed, max allowed) in the current range (min, max)
// This overload is fof the values with different current range
CVector3 ScaleBetween(CVector3 v, float minAllowed, float maxAllowed, CVector3 min, CVector3 max)
{
	return CVector3{	(maxAllowed - minAllowed) * (v.x - min.x) / (max.x - min.x) + minAllowed ,
						(maxAllowed - minAllowed) * (v.y - min.y) / (max.y - min.y) + minAllowed,
						(maxAllowed - minAllowed) * (v.z - min.z) / (max.z - min.z) + minAllowed };
}

CVector3 Reflect(const CVector3& vec, const CVector3& surfaceNormal)
{
	const auto s = Normalise(surfaceNormal);
	return vec - 2.0f * s * Dot(s, vec);
}

// Scale between two values (min allowed, max allowed) in the current range (min, max)
// This overload is fof the values with all the same current range
CVector3 ScaleBetween(CVector3 v, float minAllowed, float maxAllowed, float min, float max)
{
  return (maxAllowed - minAllowed) * (v - min) / (max - min) + minAllowed;
}
