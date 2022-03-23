#pragma once

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

using namespace std;

#include <condition_variable>
#include <thread>
#include <vector>

#include "Math/CVector2.h"
#include "Math/CVector3.h"
#include "Math/MathHelpers.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>

//---------------------------------------------------------------
// Settings
//---------------------------------------------------------------


//#define _VISUALIZATION_ON
//#define _LOG
//#define _3D


//---------------------------------------------------------------
// End Settings
//---------------------------------------------------------------


#ifdef _VISUALIZATION_ON

#include "TL-Engine.h"

using namespace tle;

I3DEngine* myEngine;
ICamera* myCamera;

#endif

struct SSphere
{

#ifdef _VISUALIZATION_ON
	IModel* mModel = nullptr;
#endif

	CVector3 mColour;
	uint8_t  mHealth = 100;
	std::string  mName;
};

struct SSphereCollisionInfo
{
#ifdef _3D
	CVector3 mVelocity;
	CVector3 mPosition;
#else
	CVector2 mVelocity;
	CVector2 mPosition;
#endif

	int indexInPartition;
	int index; // to keep track of its position in the array , negative for the blocking spheres, positive for the moving spheres
	std::vector<SSphereCollisionInfo*>* mPartition; // to keep track of the partition this sphere is in
	float mRadius;
};


//---------------------------------------------------------------------------------------------------------------------
// Thread Pools
//---------------------------------------------------------------------------------------------------------------------

// A worker thread wakes up when work is signalled to be ready, and signals back when the work is complete.
// Same condition variable is used for signalling in both directions.
// A mutex is used to guard data shared between threads
struct WorkerThread
{
	std::thread             thread;
	std::condition_variable workReady;
	std::mutex              lock;
};

// Data describing work to do by a worker thread - this task is collision detection between some sprites against some blockers
struct UpdateSpheresWork
{
	bool     complete = true;
	SSphereCollisionInfo* start; // The work is described simply as the parameters to the BlockSprites function
	SSphereCollisionInfo* end;
};



// A pool of worker threads, each with its associated work
// A more flexible system could generalise the type of work that worker threads can do
static const uint32_t                      MAX_WORKERS = 31;
std::pair<WorkerThread, UpdateSpheresWork> mUpdateSpheresWorkers[MAX_WORKERS];
uint32_t                                   mNumWorkers; // Actual number of worker threads being used in array above


struct CollisionInfoData
{
	int time;
	std::string name[2];
	uint8_t healthRemaining[2];
};

std::vector<CollisionInfoData> gCollisionInfoData;

bool bUsingMultithreading = true;

constexpr uint32_t KNumOfSpheresSQRD = 1000;
constexpr uint32_t KNumOfSpheres = KNumOfSpheresSQRD * KNumOfSpheresSQRD;
constexpr float KRangeSpawn = 500000.f;
constexpr uint32_t KNumPartitions = 100;
constexpr int kPartitionSize = (int)KRangeSpawn / KNumPartitions * 2;
constexpr float KRangeVelocity = 50.f;
constexpr float KRangeRadius = 2.f;

#ifdef _3D
CVector3 KWallBoundsMax = CVector3(KRangeSpawn, KRangeSpawn,KRangeSpawn);
CVector3 KWallBoundsMin = -KWallBoundsMax;
#else
CVector2 KWallBoundsMax = CVector2(KRangeSpawn, KRangeSpawn);
CVector2 KWallBoundsMin = -KWallBoundsMax;
#endif

// DOD approach
// Keep only the collision related information in a separate vector

SSphere gMovingSpheres[KNumOfSpheres /2];
SSphere gBlockingSpheres[KNumOfSpheres/2];

vector<SSphereCollisionInfo> gMovingSpheresCollisionInfo;
vector<SSphereCollisionInfo> gBlockingSpheresCollisionInfo;


float frameTime;
float renderingTime;
float workTime;
float totalTime = -1;
