#pragma once

#include "Common.h"



struct Grid
{


#ifdef _3D

	void Add(SSphereCollisionInfo* s)
	{
		const auto x = static_cast<int>((s->mPosition.x + KRangeSpawn) / kPartitionSize);
		const auto y = static_cast<int>((s->mPosition.y + KRangeSpawn) / kPartitionSize);
		const auto z = static_cast<int>((s->mPosition.z + KRangeSpawn) / kPartitionSize);

		mPartitions[y][x][z].push_back(s);
		s->mPartition = &mPartitions[y][x][z];
	}

	auto* GetPartition(CVector3& pos)
	{
		const auto x = static_cast<int>((pos.x + KRangeSpawn) / kPartitionSize);
		const auto y = static_cast<int>((pos.y + KRangeSpawn) / kPartitionSize);
		const auto z = static_cast<int>((pos.z + KRangeSpawn) / kPartitionSize);

		return &mPartitions[x][y][z];
	}

	std::vector<SSphereCollisionInfo*> mPartitions[KNumPartitions][KNumPartitions][KNumPartitions];

#else

	void GetNeighboursPartitions(CVector2& pos, std::vector<SSphereCollisionInfo*>* neighbours[8])
	{

		const auto x = static_cast<int>((pos.x + KRangeSpawn) / kPartitionSize);
		const auto y = static_cast<int>((pos.y + KRangeSpawn) / kPartitionSize);

		auto n = KNumPartitions - 1;

		if (x > 0 && y > 0) neighbours[0] = &mPartitions[y - 1	* KNumPartitions + x - 1];	else neighbours[0] = nullptr;
		if (y > 0)			neighbours[1] = &mPartitions[y - 1	* KNumPartitions + x    ];	else neighbours[1] = nullptr;
		if (x < n && y > 0) neighbours[2] = &mPartitions[y - 1	* KNumPartitions + x + 1];	else neighbours[2] = nullptr;
		if (x > 0)			neighbours[3] = &mPartitions[y		* KNumPartitions + x - 1];	else neighbours[3] = nullptr;
		if (x < n && y < n)	neighbours[4] = &mPartitions[y + 1	* KNumPartitions + x + 1];	else neighbours[4] = nullptr;
		if (y < n)			neighbours[5] = &mPartitions[y + 1	* KNumPartitions + x    ];	else neighbours[5] = nullptr;
		if (y < n && x > 0)	neighbours[6] = &mPartitions[y + 1	* KNumPartitions + x - 1];	else neighbours[6] = nullptr;
		if (x > 0)			neighbours[7] = &mPartitions[y		* KNumPartitions + x - 1];	else neighbours[7] = nullptr;
		
	}

	void Add(SSphereCollisionInfo* s)
	{
		const auto x = static_cast<int>((s->mPosition.x + KRangeSpawn) / kPartitionSize);
		const auto y = static_cast<int>((s->mPosition.y + KRangeSpawn) / kPartitionSize);

		mPartitions[y * KNumPartitions + x].push_back(s);
		s->indexInPartition = mPartitions[y * KNumPartitions + x].size() - 1;
		s->mPartition = &mPartitions[y * KNumPartitions + x];
	}

	auto* GetPartition(CVector2& pos)
	{
		const auto x = static_cast<int>((pos.x + KRangeSpawn) / kPartitionSize);
		const auto y = static_cast<int>((pos.y + KRangeSpawn) / kPartitionSize);

		return &mPartitions[y * KNumPartitions + x];
	}

	std::vector<SSphereCollisionInfo*> mPartitions[KNumPartitions*KNumPartitions];

#endif

}gGrid;

template<typename T>
SSphereCollisionInfo* CollisionSpatialPartitioning(SSphereCollisionInfo* sphere, T& surfaceNormal)
{

#ifdef _3D

	if (sphere->mPosition.x >= KWallBoundsMax.x ||
		sphere->mPosition.x <= KWallBoundsMin.x)
	{
		surfaceNormal = CVector3(1.f, 0.f,0.0f);
		return sphere;
	}
	if (sphere->mPosition.y >= KWallBoundsMax.y ||
		sphere->mPosition.y <= KWallBoundsMin.y)
	{
		surfaceNormal = CVector3(0.f, 1.f, 0.0f);
		return sphere;
	}

	if (sphere->mPosition.z >= KWallBoundsMax.z ||
		sphere->mPosition.z <= KWallBoundsMin.z)
	{
		surfaceNormal = CVector3(0.f, 0.f, 1.0f);
		return sphere;
	}

#else
	if (sphere->mPosition.x >= KWallBoundsMax.x ||
		sphere->mPosition.x <= KWallBoundsMin.x)
	{
		surfaceNormal = CVector2(1.f, 0.f);
		return sphere;
	}
	if (sphere->mPosition.y >= KWallBoundsMax.y ||
		sphere->mPosition.y <= KWallBoundsMin.y)
	{
		surfaceNormal = CVector2(0.f, 1.f);
		return sphere;
	}
#endif

	
		const std::vector<SSphereCollisionInfo*>* p = gGrid.GetPartition(sphere->mPosition);

		for (SSphereCollisionInfo* s : *p)
		{
			if (s == sphere) continue;

			const auto v = s->mPosition - sphere->mPosition;
			const auto mag = v.Magnitude();
			const auto rad = s->mRadius + sphere->mRadius;

			if (mag <= rad * rad * 100.f)
			{
				surfaceNormal = v * mag;
				return s;
			}

		}

		// if no collision inside the same partition, we need to check also the neighbours partitions

		std::vector<SSphereCollisionInfo*>* neighbours[8]{nullptr};
	   //gGrid.GetNeighboursPartitions(sphere->mPosition,neighbours);

	for (int i = 0; i < 8; ++i)
	{
		auto p = neighbours[i];

		if (p)
		{
			for (SSphereCollisionInfo* s : *p)
			{
				if (s == sphere) continue;

				const auto v = s->mPosition - sphere->mPosition;
				const auto mag = v.Magnitude();
				const auto rad = s->mRadius + sphere->mRadius;

				if (mag <= rad * rad * 100.f)
				{
					surfaceNormal = v * mag;
					return s;
				}
			}
		}
	}
	
	return nullptr;
}




template<typename T>
inline SSphereCollisionInfo* CollisionLineSweep(SSphereCollisionInfo* sphere, T& surfaceNormal)
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
		m = s + (int)((e - s) * .5);
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

template<typename T>
inline SSphereCollisionInfo* Collision(SSphereCollisionInfo* sphere, T& surfaceNormal)
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
	auto blockersEnd = s + gBlockingSpheresCollisionInfo.size();

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
