#pragma once

#include "Common.h"
#include "Math/CVector2.h"


bool CollisionLineSweep(SSphere* sphere, SSphere* blockersStart, SSphere* blockersEnd, CVector2& surfaceNormal)
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
		return  true;
	}

	if (sphere->mPosition.y >= KWallBoundsMax.y ||
		sphere->mPosition.y <= KWallBoundsMin.y)
	{
		surfaceNormal = CVector2(0.f, 1.f);
		sphere->mVelocity = Reflect(sphere->mVelocity, surfaceNormal);
		return true;
	}


	//////////////////////////////
	///
	///	Line Sweep with blockers
	///
	//////////////////////////////

	bool found = false;
	auto s = blockersStart;
	auto e = blockersEnd;

	auto sr = sphere->mPosition.x + sphere->mRadius;

	SSphere* m;


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
				b->mHealth -= 20;
				sphere->mHealth -= 20;
				return true;
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
				b->mHealth -= 20;
				sphere->mHealth -= 20;
				return true;
			}

		}
	}


	//////////////////////////////
	///
	///	TODO: Between moving spheres
	///
	//////////////////////////////

	auto sp = gMovingSpheres.data();
	auto spEnd = sp + gMovingSpheres.size();

	while (sp != spEnd)
	{
		if (sp == sphere) { ++sp; continue; }

		const auto v = sp->mPosition - sphere->mPosition;
		const auto mag = v.Magnitude();
		const auto rad = sp->mRadius + sphere->mRadius;

		if (mag <= rad * rad * 100)
		{
			surfaceNormal = CVector2(v.x * mag, v.y * mag);
			sp->mHealth -= 20;
			sphere->mHealth -= 20;
			return true;
		}
		++sp;
	}


	return false;
}



bool Collision(SSphere* sphere, SSphere* blockersStart, SSphere* blockersEnd, CVector2& surfaceNormal)
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
		return  true;
	}

	if (sphere->mPosition.y >= KWallBoundsMax.y ||
		sphere->mPosition.y <= KWallBoundsMin.y)
	{
		surfaceNormal = CVector2(0.f, 1.f);
		return true;
	}


	//////////////////////////////
	///
	///	Collision with blockers
	///
	//////////////////////////////
	
	auto s = blockersStart;
	auto b = s;
	while (b != blockersEnd)
	{
		if (b == sphere)
		{
			++b;
			continue;
		}

		const auto v = b->mPosition - sphere->mPosition;
		const auto mag = v.Magnitude();
		const auto rad = b->mRadius + sphere->mRadius;

		if (mag <= rad * rad * 100.f)
		{
			surfaceNormal = CVector2(v.x * mag, v.y * mag);
			b->mHealth -= 20;
			sphere->mHealth -= 20;
			return true;
		}
		++b;
	}


	//////////////////////////////
	///
	///	TODO: Between moving spheres
	///
	//////////////////////////////

	auto sp = gMovingSpheres.data();
	auto spEnd = sp + gMovingSpheres.size();

	while (sp != spEnd)
	{
		if (sp == sphere) { ++sp; continue; }

		const auto v = sp->mPosition - sphere->mPosition;
		const auto mag = v.Magnitude();
		const auto rad = sp->mRadius + sphere->mRadius;

		if (mag <= rad * rad * 100)
		{
			surfaceNormal = CVector2(v.x * mag, v.y * mag);
			sp->mHealth -= 20;
			sphere->mHealth -= 20;
			return true;
		}
		++sp;
	}


	return false;
}

