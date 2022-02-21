// Assignment.cpp: A program using the TL-Engine

#include <TL-Engine.h>	// TL-Engine include file and namespace

#include "thread_pool.h"

#include "Math/CVector2.h"
#include "Math/CVector3.h"
#include "Math/MathHelpers.h"

using namespace tle;

I3DEngine* myEngine;

struct SSphere
{
	IModel* mModel = nullptr;
	TFloat32 mRadius = 1.f;
	CVector2 mVelocity;
	CVector2 mPosition;
	CVector3 mColour;
	uint8_t mHealth = 100;
	char mName[];
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
	bool complete = true;
	SSphere* spheres; // The work is described simply as the parameters to the BlockSprites function
	uint32_t numSpheres;
	SSphere* blockers;
	uint32_t numBlockers;
};

// A pool of worker threads, each with its associated work
// A more flexible system could generalise the type of work that worker threads can do
static const uint32_t MAX_WORKERS = 31;
std::pair<WorkerThread, UpdateSpheresWork> mUpdateSpheresWorkers[MAX_WORKERS];
uint32_t mNumWorkers;  // Actual number of worker threads being used in array above



std::vector<SSphere> gSpheres;

thread_pool TPool;

bool bUsingMultithreading = true;

constexpr uint32_t KNumOfSpheres = 1000;
constexpr TFloat32 KRangeSpawn = 2000.f;
constexpr TFloat32 KRangeVelocity = 100.f;
constexpr TFloat32 KRangeRadius = 2.f;

CVector2 KWallBoundsMax = CVector2(KRangeSpawn, KRangeSpawn);
CVector2 KWallBoundsMin = -KWallBoundsMax;

ICamera* myCamera;

float frameTime;

bool SceneSetup()
{
	myCamera = myEngine->CreateCamera(kManual, 0.0f, 0.f, -2000.f);

	gSpheres.reserve(KNumOfSpheres);

	const auto sphereMesh = myEngine->LoadMesh("sphere.x");

	for (auto i = 0u; i < KNumOfSpheres; ++i)
	{
		SSphere s;

		s.mPosition = CVector2::Rand() * KRangeSpawn;

		s.mModel = sphereMesh->CreateModel();

		if (i < KNumOfSpheres / 2)
		{
			s.mVelocity = CVector2::Rand() * KRangeVelocity;
		}
		else
		{
			s.mVelocity = CVector2(0.f, 0.f);
		}

		s.mRadius = Random(0.5f, KRangeRadius);

		s.mModel->Scale(s.mRadius);

		gSpheres.push_back(s);
	}

	return true;
}


void Work(SSphere* start, SSphere* end)
{
	auto sphere = start;

	while (sphere != end)
	{

		//	// remove this
	//	if(sphere->mHealth<=0)
	//	{
	//		// Copy the particle at the end of the collection over the current (dead) particle
	//		// Do this for each collection of data. Don't step the current pointer forward.
	//		--end;
	//		*sphere = *end;

	//		continue;
	//	}

		bool collided = false;
		CVector2 surfaceNormal;

		if (sphere->mPosition.x >= KWallBoundsMax.x ||
			sphere->mPosition.x <= KWallBoundsMin.x)
		{
			surfaceNormal = CVector2(1.f, 0.f);
			collided = true;
		}
		else if (sphere->mPosition.y >= KWallBoundsMax.y ||
				sphere->mPosition.y <= KWallBoundsMin.y)
		{
			surfaceNormal = CVector2(0.f, 1.f);
			collided = true;
		}

		if (!collided)
		{
			const auto othersStart = gSpheres.data();
			const auto othersEnd = othersStart + gSpheres.size();
			auto       other = othersStart;

			while (other != othersEnd)
			{
				if (other != sphere)
				{
					const auto v = other->mPosition - sphere->mPosition;
					const auto mag = v.Magnitude();
					const auto rad = other->mRadius + sphere->mRadius;
					if (mag <= rad * rad * 100)
					{
						const auto n = CVector2(v.x / mag, v.y / mag);
						surfaceNormal = n;
						collided = true;
						other->mHealth -= 20;
						sphere->mHealth -= 20;
						break;
					}
				}
				other++;
			}
		}

		if (collided)
		{
			sphere->mVelocity = Reflect(sphere->mVelocity, surfaceNormal);
		}

		sphere->mPosition += sphere->mVelocity * frameTime;
		sphere->mModel->SetPosition(sphere->mPosition.x, sphere->mPosition.y, 0.0f);

		sphere++;
	}

	//// Resize arrays after any deletions
	//uint32_t newSize = static_cast<uint32_t>(end - gSpheres.data());
	//gSpheres.resize(newSize);
}


void UpdateThread(uint32_t thread)
{
	auto& worker = mUpdateSpheresWorkers[thread].first;
	auto& work = mUpdateSpheresWorkers[thread].second;
	while (true)
	{
		{
			std::unique_lock<std::mutex> l(worker.lock);
			worker.workReady.wait(l, [&]() { return !work.complete; });
		}

		// We have some work so do it...
		Work(work.spheres, work.spheres + work.numSpheres);

		{
			// Flag the work is complete
			// We also guard every normal access to shared variable "work.complete" with the same mutex
			std::unique_lock<std::mutex> l(worker.lock);
			work.complete = true;
		}
		// Send a signal back to the main thread to say the work is complete, loop back and wait for more work
		worker.workReady.notify_one();
	}
}

void UpdateSpheres()
{
	if (bUsingMultithreading)
	{
		const auto nThreads = TPool.n_threads();
		const auto numSpheres = gSpheres.size();
		const auto spheresPerSection = numSpheres / (nThreads + 1);

		auto spheres = gSpheres.data();

		for (int i = 0; i < nThreads; ++i)
		{
			auto& work = mUpdateSpheresWorkers[i].second;

			work.numSpheres = spheresPerSection;
			work.spheres = spheres;

			// Flag the work as not yet complete
			auto& workerThread = mUpdateSpheresWorkers[i].first;
			{
				// Guard every access to shared variable "work.complete" with a mutex (see BlockSpritesThread comments)
				std::unique_lock<std::mutex> l(workerThread.lock);
				work.complete = false;
			}

			// Notify the worker thread via a condition variable - this will wake the worker thread up
			workerThread.workReady.notify_one();

			spheres += spheresPerSection;
		}

		// do the remaining work
		Work(spheres, spheres + spheresPerSection);

		// Wait for all the workers to finish
		for (uint32_t i = 0; i < mNumWorkers; ++i)
		{
			auto& workerThread = mUpdateSpheresWorkers[i].first;
			auto& work = mUpdateSpheresWorkers[i].second;

			// Wait for a signal via a condition variable indicating that the worker is complete
			// See comments in BlockSpritesThread regarding the mutex and the wait method
			std::unique_lock<std::mutex> l(workerThread.lock);
			workerThread.workReady.wait(l, [&]() { return work.complete; });
		}
	}
	else
	{
		Work(&gSpheres[0], &gSpheres[gSpheres.size() - 1]);
	}
}


bool GameLoop()
{
	UpdateSpheres();

	myCamera->MoveZ(myEngine->GetMouseWheelMovement() * 100);
	if (myEngine->KeyHit(Key_Escape)) return false;
	if (myEngine->KeyHit(Key_Space)) bUsingMultithreading = !bUsingMultithreading;

	return true;
}



void main()
{
	srand(time(0));

	//---------------------------------------------------------------------------------------------------------------------

	//*********************************************************
	// Start worker threads
	mNumWorkers = std::thread::hardware_concurrency(); // Gives a hint about level of thread concurrency supported by system (0 means no hint given)
	if (mNumWorkers == 0)  mNumWorkers = 8;
	--mNumWorkers; // Decrease by one because this main thread is already running
	for (uint32_t i = 0; i < mNumWorkers; ++i)
	{
		// Start each worker thread running the work method. Note the way to construct std::thread to run a member function
		mUpdateSpheresWorkers[i].first.thread = std::thread(&UpdateThread, i);
	}


	// Create a 3D engine (using TLX engine here) and open a window for it
	myEngine = New3DEngine(kTLXLegacy);
	myEngine->StartWindowed(1920, 1080);

	// Add default folder for meshes and other media
	myEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");

	/**** Set up your scene here ****/

	SceneSetup();
	
	const auto font = myEngine->DefaultFont();

	// The main game loop, repeat until engine is stopped
	while (myEngine->IsRunning())
	{
		// Draw the scene
		myEngine->DrawScene();

		/**** Update your scene each frame here ****/

		if (!GameLoop()) break;

		static float t = .3f;

		if (t < 0.f)
		{
			frameTime = myEngine->Timer();
			t = .3f;
		}
		else t -= myEngine->Timer();

		font->Draw("ms: " + std::to_string(frameTime), 10, 10);
		font->Draw("FPS: " + std::to_string(1.0f / frameTime), 10, 20);
		font->Draw("Multi threading: " + std::string(bUsingMultithreading ? "Yes" : "No"), 10, 30);
	}

	for (uint32_t i = 0; i < mNumWorkers; ++i)
	{
		// Start each worker thread running the work method. Note the way to construct std::thread to run a member function
		mUpdateSpheresWorkers[i].first.thread.detach();
	}


	// Delete the 3D engine now we are finished with it
	myEngine->Delete();
}
