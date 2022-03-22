//--------------------------------------------------------------------------------------
// Vector2 class (cut down version), mainly used for texture coordinates (UVs)
// but can be used for 2D points as well
//--------------------------------------------------------------------------------------

#include "CVector2.h"

#include "MathHelpers.h"

/*-----------------------------------------------------------------------------------------
	Operators
-----------------------------------------------------------------------------------------*/

float* CVector2::GetValuesArray()
{
	return &x;
}

// Addition of another vector to this one, e.g. Position += Velocity
CVector2& CVector2::operator+= (const CVector2& v)
{
	x += v.x;
	y += v.y;
	return *this;
}

// Subtraction of another vector from this one, e.g. Velocity -= Gravity
CVector2& CVector2::operator-= (const CVector2& v)
{
	x -= v.x;
	y -= v.y;
	return *this;
}

// Negate this vector (e.g. Velocity = -Velocity)
CVector2 CVector2::operator- () noexcept
{
	return CVector2(-x, -y);
}

// Plus sign in front of vector - called unary positive and usually does nothing. Included for completeness (e.g. Velocity = +Velocity)
CVector2& CVector2::operator+ ()
{
	return *this;
}

// Multiply vector by scalar (scales vector);
CVector2& CVector2::operator*= (float s)
{
	x *= s;
	y *= s;
	return *this;
}

CVector2& CVector2::operator%=(float s)
{
	int x1 = x;
	int y1 = y;
	float x2 = x - x1;
	float y2 = y - y1;
	x1 %= (int)s;
	y1 %= (int)s;
	x = (float)x1 + x2;
	y = (float)y1 + y2;
	return *this;
}

CVector2& CVector2::operator%=(int s)
{
	int x1 = x;
	int y1 = y;
	float x2 = x - x1;
	float y2 = y - y1;
	x1 %= s;
	y1 %= s;
	x = (float) x1 + x2;
	y = (float) y1 + y2;
	return *this;
}



float CVector2::Magnitude() const
{
	return x * x + y * y;
}

float CVector2::MagnitudeSqrt() const
{
	return std::sqrt(x * x + y * y);
}

// Instead of calculating the magnitude of the vector, calculate the inverse square root using the Quake3 inverse square root algorithm
float CVector2::InverseMagnitude() const
{
	return Q_rsqrt(x * x + y * y);
}


CVector2 CVector2::Rand()
{
	return CVector2(Random(-1.0f,1.0f), Random(-1.0f,1.0f));
}

CVector2 CVector2::Up()
{
	return CVector2(.0f, 1.f);
}

CVector2 CVector2::Right()
{
	return CVector2(1.f, 0.f);
}

CVector2 CVector2::Down()
{
	return CVector2(.0f, -1.f);
}

CVector2 CVector2::Left()
{
	return CVector2(-1.f, 0.f);
}


// Vector-vector addition
CVector2 operator+ (const CVector2& v, const CVector2& w)
{
	return { v.x + w.x, v.y + w.y };
}

// Vector-vector subtraction
CVector2 operator- (const CVector2& v, const CVector2& w)
{
	return { v.x - w.x, v.y - w.y };
}

// Vector-scalar multiplication
CVector2 operator* (const CVector2& v, float s)
{
	return { v.x * s, v.y * s };
}

CVector2 operator*(const CVector2& v,const CVector2& w)
	{
	return { v.x * w.x, v.y * w.y };
	}

CVector2 operator* (float s, const CVector2& v)
{
	return { v.x * s, v.y * s };
}

// Vector-scalar division
CVector2 operator/ (const CVector2& v, float s)
{
	return { v.x / s, v.y / s };
}

/*-----------------------------------------------------------------------------------------
	Non-member functions
-----------------------------------------------------------------------------------------*/

// Dot product of two given vectors (order not important) - non-member version
float Dot(const CVector2& v1, const CVector2& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

// Return unit length vector in the same direction as given one
CVector2 Normalise(const CVector2& v)
{
	float lengthSq = v.x * v.x + v.y * v.y;

	// Ensure vector is not zero length (use function from MathHelpers.h to check if float is approximately 0)
	if (IsZero(lengthSq))
	{
		return { 0.0f, 0.0f };
	}
	else
	{
		const float invLength = InvSqrt(lengthSq);
		return { v.x * invLength, v.y * invLength };
	}
}

CVector2 Reflect(const CVector2& vec, const CVector2& surfaceNormal)
{
	const auto s = Normalise(surfaceNormal);
	return vec - 2.0f * s * Dot(s, vec);
}

bool IsZero(const CVector2& v)
{
	return std::abs(v.x) + std::abs(v.y) < EPSILON;
}
