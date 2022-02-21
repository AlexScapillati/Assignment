#pragma once

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

//#define _VISUALIZATION_ON

#include <condition_variable>
#include <thread>
#include <vector>

#include "thread_pool.h"
#include "Math/CVector2.h"
#include "Math/CVector3.h"

#ifdef _VISUALIZATION_ON

using namespace tle;

I3DEngine* myEngine;
ICamera* myCamera;

#endif


struct SSphere
{

#ifdef _VISUALIZATION_ON
	IModel* mModel = nullptr;
#endif
	float mRadius;
	CVector2 mVelocity;
	CVector2 mPosition;
	CVector3 mColour;
	uint8_t  mHealth = 100;
	char     mName[];
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
	SSphere* spheres; // The work is described simply as the parameters to the BlockSprites function
	uint32_t numSpheres;
	SSphere* blockers;
	uint32_t numBlockers;
};

// A pool of worker threads, each with its associated work
// A more flexible system could generalise the type of work that worker threads can do
static const uint32_t                      MAX_WORKERS = 31;
std::pair<WorkerThread, UpdateSpheresWork> mUpdateSpheresWorkers[MAX_WORKERS];
uint32_t                                   mNumWorkers; // Actual number of worker threads being used in array above


vector<SSphere> gMovingSpheres;
std::vector<SSphere> gBlockingSpheres;

thread_pool TPool;


bool bUsingMultithreading = true;

constexpr uint32_t KNumOfSpheres = 1000;
constexpr float KRangeSpawn = 2000.f;
constexpr float KRangeVelocity = .05f;
constexpr float KRangeRadius = 2.f;

CVector2 KWallBoundsMax = CVector2(KRangeSpawn, KRangeSpawn);
CVector2 KWallBoundsMin = -KWallBoundsMax;


float frameTime;
float renderingTime;
float workTime;
float totalTime = -1;
