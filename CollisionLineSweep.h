#pragma once

#include "Common.h"


SSphereCollisionInfo* CollisionLineSweep(SSphereCollisionInfo* sphere, CVector2& surfaceNormal)
{
	//////////////////////////////
	///
	///	Check Edges
	///
	//////////////////////////////


	if (sphere->mPosition.x >= KWallBoundsMax.x ||
		sphere->mPosition.x <= KWallBoundsMin.x)
	{
		surfaceNormal = CVector2(1.f, 0.f);
		sphere->mVelocity = Reflect(sphere->mVelocity, surfaceNormal);
		return  nullptr;
	}

	if (sphere->mPosition.y >= KWallBoundsMax.y ||
		sphere->mPosition.y <= KWallBoundsMin.y)
	{
		surfaceNormal = CVector2(0.f, 1.f);
		sphere->mVelocity = Reflect(sphere->mVelocity, surfaceNormal);
		return nullptr;
	}


	//////////////////////////////
	///
	///	Line Sweep with blockers
	///
	//////////////////////////////

	auto blockersStart = gBlockingSpheresCollisionInfo.data();
	auto blockersEnd = blockersStart + gBlockingSpheresCollisionInfo.size();


	bool found = false;
	auto s = blockersStart;
	auto e = blockersEnd;

	auto sr = sphere->mPosition.x + sphere->mRadius;

	SSphereCollisionInfo* m;


	do
	{
		m = s + (e - s) / 2;
		if (sr <= m->mPosition.x) e = m;
		else if (sr <= m->mPosition.x) s = m;
		else						   found = true;
	} while (!found && e - s > 1);


	if (found)
	{

		auto b = m;
		while (b != blockersEnd && sr > b->mPosition.x + b->mRadius)
		{
			if (b == sphere)
			{
				++b; continue;
			}


			const auto v = b->mPosition - sphere->mPosition;
			const auto mag = v.Magnitude();
			const auto rad = b->mRadius + sphere->mRadius;

			if (mag <= rad * rad * 100.f)
			{
				surfaceNormal = CVector2(v.x * mag, v.y * mag);
				return b;
			}++b;
		}

		b = m;
		while (b-- != blockersStart && sr < b->mPosition.x - b->mRadius)
		{
			if (b == sphere)
			{
				continue;
			}

			const auto v = b->mPosition - sphere->mPosition;
			const auto mag = v.Magnitude();
			const auto rad = b->mRadius + sphere->mRadius;

			if (mag <= rad * rad * 100)
			{
				surfaceNormal = CVector2(v.x * mag, v.y * mag);
				return b;
			}

		}
	}


	//////////////////////////////
	///
	///	TODO: Between moving spheres
	///
	//////////////////////////////

	auto sp = gMovingSpheresCollisionInfo.data();
	auto spEnd = sp + gMovingSpheresCollisionInfo.size();

	while (sp != spEnd)
	{
		if (sp == sphere) { ++sp; continue; }

		const auto v = sp->mPosition - sphere->mPosition;
		const auto mag = v.Magnitude();
		const auto rad = sp->mRadius + sphere->mRadius;

		if (mag <= rad * rad * 100)
		{
			surfaceNormal = CVector2(v.x * mag, v.y * mag);
			return sp;
		}
		++sp;
	}


	return nullptr;
}

inline SSphereCollisionInfo* Collision(SSphereCollisionInfo* sphere,CVector2& surfaceNormal)
{
	//////////////////////////////
	///
	///	Check Edges
	///
	//////////////////////////////


	if (sphere->mPosition.x >= KWallBoundsMax.x ||
		sphere->mPosition.x <= KWallBoundsMin.x)
	{
		surfaceNormal = CVector2(1.f, 0.f);
		return  sphere;
	}

	if (sphere->mPosition.y >= KWallBoundsMax.y ||
		sphere->mPosition.y <= KWallBoundsMin.y)
	{
		surfaceNormal = CVector2(0.f, 1.f);
		return sphere;
	}


	//////////////////////////////
	///
	/// Collision with blockers
	///
	//////////////////////////////

	auto s = gBlockingSpheresCollisionInfo.data();
	auto blockersEnd =  s + gBlockingSpheresCollisionInfo.size();

	auto sr = sphere->mPosition.x + sphere->mRadius;

	while (s != blockersEnd && sr > s->mPosition.x + s->mRadius)
	{
		if (s == sphere)
		{
			++s; continue;
		}


		const auto v = s->mPosition - sphere->mPosition;
		const auto mag = v.Magnitude();
		const auto rad = s->mRadius + sphere->mRadius;

		if (mag <= rad * rad * 100.f)
		{
			surfaceNormal = CVector2(v.x * mag, v.y * mag);
			return s;
		}

		++s;
	}




	//////////////////////////////
	///
	///	TODO: Between moving spheres
	///
	//////////////////////////////

	auto sp = gMovingSpheresCollisionInfo.data();
	auto spEnd = sp + gMovingSpheresCollisionInfo.size();

	while (sp != spEnd)
	{
		if (sp == sphere) { ++sp; continue; }

		const auto v = sp->mPosition - sphere->mPosition;
		const auto mag = v.Magnitude();
		const auto rad = sp->mRadius + sphere->mRadius;

		if (mag <= rad * rad * 100)
		{
			surfaceNormal = CVector2(v.x * mag, v.y * mag);
			return sp;
		}
		++sp;
	}


	return nullptr;
}

