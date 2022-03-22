// Assignment.cpp: A program using the TL-Engine

#include <algorithm>
#include <fstream>
#include <iostream>
#include <TL-Engine.h>	// TL-Engine include file and namespace

#include "CollisionLineSweep.h"

#include "SpatialPartitioning.h"
#include "Math/CVector2.h"
#include "Math/MathHelpers.h"

Grid gGrid(kNumCells, kCellSize);

bool SceneSetup()
{

	gCollisionInfoData.erase(gCollisionInfoData.begin(),gCollisionInfoData.end());
	gMovingSpheres.erase(gMovingSpheres.begin(), gMovingSpheres.end());
	gBlockingSpheres.erase(gBlockingSpheres.begin(), gBlockingSpheres.end());

	gCollisionInfoData.reserve(KNumOfSpheres * 2);
	gMovingSpheres.reserve(KNumOfSpheres / 2);
	gBlockingSpheres.reserve(KNumOfSpheres / 2);


	gMovingSpheresCollisionInfo.erase(gMovingSpheresCollisionInfo.begin(), gMovingSpheresCollisionInfo.end());
	gBlockingSpheresCollisionInfo.erase(gBlockingSpheresCollisionInfo.begin(), gBlockingSpheresCollisionInfo.end());

	gMovingSpheresCollisionInfo.reserve(KNumOfSpheres / 2);
	gBlockingSpheresCollisionInfo.reserve(KNumOfSpheres / 2);


#ifdef _VISUALIZATION_ON
	myCamera = myEngine->CreateCamera(kManual, 0.0f, 0.f, -2000.f);
	const auto sphereMesh = myEngine->LoadMesh("sphere.x");
	const auto blockedMesh = myEngine->LoadMesh("SphereBlocked.x");
#endif

	for (auto i = 0u; i < KNumOfSpheres; ++i)
	{
		SSphereCollisionInfo s;
		SSphere ss;

		s.mRadius = Random(0.5f, KRangeRadius);
		ss.mColour = CVector3::Rand();
		ss.mName = std::to_string(i);

		if (i >= KNumOfSpheres / 2)
		{
			s.mVelocity = CVector2::Rand() * KRangeVelocity;

#ifdef _VISUALIZATION_ON
			ss.mModel = sphereMesh->CreateModel();
			ss.mModel->Scale(s.mRadius);
#endif
			gMovingSpheres.push_back(ss);
			gMovingSpheresCollisionInfo.push_back(s);
		}
		else
		{
			s.mVelocity = CVector2(0.f, 0.f);

#ifdef _VISUALIZATION_ON
			ss.mModel = blockedMesh->CreateModel();
			ss.mModel->Scale(s.mRadius);
			ss.mModel->SetPosition(s.mPosition.x, s.mPosition.y, 0.0f);
#endif

			gBlockingSpheres.push_back(ss);
			gBlockingSpheresCollisionInfo.push_back(s);
			gGrid.AddSphere(&s);
		}
	}


	CVector2 null;

	
	for (auto i = 0u; i < KNumOfSpheres / 2; ++i)
	{

		auto& s = gBlockingSpheresCollisionInfo[i];

			s.mPosition = CVector2::Rand() * KRangeSpawn;
			s.mPosition += CVector2::Rand();
			s.mPosition %= KRangeSpawn;
			s.index = -i;
	}

	for (auto i = 0u; i < KNumOfSpheres / 2; ++i)
	{

		auto& s = gMovingSpheresCollisionInfo[i];
		
			s.mPosition = CVector2::Rand() * KRangeSpawn;
			s.mPosition += CVector2::Rand();
			s.mPosition %= KRangeSpawn;
			s.index = 1;
	}

	std::sort(gBlockingSpheresCollisionInfo.begin(), gBlockingSpheresCollisionInfo.end(), [](const SSphereCollisionInfo& a, const SSphereCollisionInfo& b) { return a.mPosition.x < b.mPosition.x; });





	return true;
}


void log(SSphere* first, SSphere* second)
{
	CollisionInfoData c;
	c.healthRemaining[0] = first->mHealth;
	c.healthRemaining[1] = second->mHealth;
	c.name[0] = first->mName;
	c.name[1] = second->mName;
	c.time = time(0);

	gCollisionInfoData.push_back(c);
}

void PrintLog()
{
	ofstream file;

	file.open("Output.txt");

	for (const auto& i : gCollisionInfoData)
	{
		std::string s = "[" + std::to_string(i.time) + "] Collision: " + i.name[0] + ", Health : " + std::to_string(i.healthRemaining[0]) + " with "
			+ i.name[1] + ", Health : " + std::to_string(i.healthRemaining[1]) ;

		file << s << endl;
	}

	gCollisionInfoData.clear();

	file.close();
}


void Work(SSphereCollisionInfo* start, SSphereCollisionInfo* end)
{
	auto sphere = start;
	while (sphere != end)
	{
		CVector2 surfaceNormal;

		auto c = CollisionLineSweep(sphere, surfaceNormal);

		if (c)
		{
			sphere->mPosition -= sphere->mVelocity * totalTime ;
			sphere->mVelocity = Reflect(sphere->mVelocity, surfaceNormal);

			auto j = c->index;
			auto i = sphere->index;

			if(j < 0) 
				log(&gMovingSpheres[i], &gBlockingSpheres[std::abs(j)]);
			else
				log(&gMovingSpheres[i], &gMovingSpheres[std::abs(j)]);
		}
		else
			sphere->mPosition += sphere->mVelocity * totalTime;
		
		++sphere;
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
		const auto nThreads = std::thread::hardware_concurrency();
		const auto numSpheres = gMovingSpheresCollisionInfo.size();
		const auto spheresPerSection = numSpheres / (nThreads + 1);

		auto       spheres = gMovingSpheresCollisionInfo.data();

		for (uint32_t i = 0; i < nThreads; ++i)
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
		Work(&gMovingSpheresCollisionInfo[0], &gMovingSpheresCollisionInfo[gMovingSpheresCollisionInfo.size() - 1]);
	}
}


bool GameLoop()
{
	UpdateSpheres();


#ifdef _VISUALIZATION_ON

	workTime = myEngine->Timer();

	for (int i = 0u; i < KNumOfSpheres / 2; ++i)
	{
		gBlockingSpheres[i].mModel->SetPosition(gBlockingSpheresCollisionInfo[i].mPosition.x,gBlockingSpheresCollisionInfo[i].mPosition.y,0.0f);
	}


	for (int i = 0u; i < KNumOfSpheres / 2; ++i)
	{
		gMovingSpheres[i].mModel->SetPosition(gMovingSpheresCollisionInfo[i].mPosition.x, gMovingSpheresCollisionInfo[i].mPosition.y, 0.0f);
	}

	myCamera->MoveZ(myEngine->GetMouseWheelMovement() * 100);
	myCamera->MoveLocalY(myEngine->KeyHeld(Key_W) * 1000.0f* totalTime);
	myCamera->MoveLocalY(myEngine->KeyHeld(Key_S) * -1000.f* totalTime);
	myCamera->MoveLocalX(myEngine->KeyHeld(Key_D) * 1000.0f* totalTime);
	myCamera->MoveLocalX(myEngine->KeyHeld(Key_A) * -1000.f* totalTime);
	if (myEngine->KeyHit(Key_Escape)) return false;
	if (myEngine->KeyHit(Key_Space)) bUsingMultithreading = !bUsingMultithreading;

#endif

	return true;
}




void main()
{

	srand(time(0));

	//---------------------------------------------------------------------------------------------------------------------

	//*********************************************************
	// Start worker threads
	mNumWorkers = std::thread::hardware_concurrency(); // Gives a hint about level of thread concurrency supported by system (0 means no hint given)
	if (mNumWorkers == 0) mNumWorkers = 8;
	--mNumWorkers; // Decrease by one because this main thread is already running
	for (uint32_t i = 0; i < mNumWorkers; ++i)
	{
		// Start each worker thread running the work method. Note the way to construct std::thread to run a member function
		mUpdateSpheresWorkers[i].first.thread = std::thread(&UpdateThread, i);
	}

#ifdef _VISUALIZATION_ON

	// Create a 3D engine (using TLX engine here) and open a window for it
	myEngine = New3DEngine(kTLXLegacy);
	myEngine->StartWindowed(1920, 1080);

	// Add default folder for meshes and other media
	myEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");
	const auto font = myEngine->DefaultFont();
#else
	while (true)
	{
#endif
		SceneSetup();


#ifdef _VISUALIZATION_ON
		// The main game loop, repeat until engine is stopped
		while (myEngine->IsRunning())
#else
		//for (int i = 0; i < 10000; ++i)
		//while(1)

		auto begin = chrono::steady_clock::now();
#endif
		{


#ifdef _VISUALIZATION_ON

			frameTime = myEngine->Timer();
			font->Draw("ms: " + std::to_string(totalTime), 10, 10);
			font->Draw("FPS: " + std::to_string(1.0f / totalTime), 10, 20);
			font->Draw("Work time: " + std::to_string(workTime), 10, 30);
			font->Draw("Render time: " + std::to_string(renderingTime), 10, 40);
			font->Draw("Multi threading: " + std::string(bUsingMultithreading ? "Yes" : "No"), 10, 50);

			// Draw the scene
			myEngine->DrawScene();

			renderingTime = myEngine->Timer();
#endif
			/**** Update your scene each frame here ****/

			if (!GameLoop()) break;

			PrintLog();

			if (totalTime < 0) totalTime = frameTime + renderingTime + workTime;

			static float t = .1f;
			if (t < 0.f)
			{
				totalTime = frameTime + renderingTime + workTime;
				t = .1f;
			}
			else t -= totalTime;
		}

#ifdef _VISUALIZATION_ON
		// Delete the 3D engine now we are finished with it
		myEngine->Delete();
#else
		auto end = chrono::steady_clock::now();
		cout << "Time took = " << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << "[ms]" << endl;


		}
#endif
	for (uint32_t i = 0; i < mNumWorkers; ++i)
	{
		// Start each worker thread running the work method. Note the way to construct std::thread to run a member function
		mUpdateSpheresWorkers[i].first.thread.detach();
	}

	}
