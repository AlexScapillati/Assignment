// Assignment.cpp: A program using the TL-Engine

#include <iostream>
#include <TL-Engine.h>	// TL-Engine include file and namespace

#include "CollisionLineSweep.h"
#include "Common.h"
#include "thread_pool.h"

#include "Math/CVector2.h"
#include "Math/MathHelpers.h"


bool SceneSetup()
{
	gMovingSpheres.erase(gMovingSpheres.begin(), gMovingSpheres.end());
	gBlockingSpheres.erase(gBlockingSpheres.begin(), gBlockingSpheres.end());

	gMovingSpheres.reserve(KNumOfSpheres / 2);
	gBlockingSpheres.reserve(KNumOfSpheres / 2);

#ifdef _VISUALIZATION_ON
	myCamera = myEngine->CreateCamera(kManual, 0.0f, 0.f, -2000.f);
	const auto sphereMesh = myEngine->LoadMesh("sphere.x");
	const auto blockedMesh = myEngine->LoadMesh("SphereBlocked.x");
#endif

	for (auto i = 0u; i < KNumOfSpheres; ++i)
	{
		SSphere s;

		s.mRadius = Random(0.5f, KRangeRadius);

		if (i >= KNumOfSpheres / 2)
		{
			s.mVelocity = CVector2::Rand() * KRangeVelocity;

#ifdef _VISUALIZATION_ON
			s.mModel = sphereMesh->CreateModel();
			s.mModel->Scale(s.mRadius);
#endif
			gMovingSpheres.push_back(s);
		}
		else
		{
			s.mVelocity = CVector2(0.f, 0.f);

#ifdef _VISUALIZATION_ON
			s.mModel = blockedMesh->CreateModel();
			s.mModel->Scale(s.mRadius);
			s.mModel->SetPosition(s.mPosition.x, s.mPosition.y, 0.0f);
#endif

			gBlockingSpheres.push_back(s);
		}
	}


	CVector2 null;

	for (auto& s : gMovingSpheres)
	{
		do
		{
			s.mPosition = CVector2::Rand() * KRangeSpawn;
			s.mPosition += CVector2::Rand();
			s.mPosition %= KRangeSpawn;
		} while (CollisionLineSweep(&s, gBlockingSpheres.data(), gBlockingSpheres.data() + gBlockingSpheres.size(), null));
#ifdef _VISUALIZATION_ON
			s.mModel->SetPosition(s.mPosition.x, s.mPosition.y, 0.0f);
#endif
	}


	for (auto& s : gBlockingSpheres)
	{
		do
		{
			s.mPosition = CVector2::Rand() * KRangeSpawn;
			s.mPosition += CVector2::Rand();
			s.mPosition %= KRangeSpawn;
		} while (CollisionLineSweep(&s, gBlockingSpheres.data(), gBlockingSpheres.data() + gBlockingSpheres.size(), null));
#ifdef _VISUALIZATION_ON
			s.mModel->SetPosition(s.mPosition.x, s.mPosition.y, 0.0f);
#endif
	}

	std::sort(gBlockingSpheres.begin(), gBlockingSpheres.end(), [](const SSphere& a, const SSphere& b) { return a.mPosition.x < b.mPosition.x; });

	return true;
}


void Work(SSphere* start, SSphere* end, SSphere* blockersStart, SSphere* blockersEnd)
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


		CVector2 surfaceNormal;
		
		if (Collision(sphere, blockersStart, blockersEnd, surfaceNormal))
		{
			sphere->mPosition -= sphere->mVelocity * totalTime * 4.f;
			sphere->mVelocity = Reflect(sphere->mVelocity, surfaceNormal);
		}
		else
			sphere->mPosition += sphere->mVelocity * totalTime;
		

#ifdef _VISUALIZATION_ON
		sphere->mModel->SetPosition(sphere->mPosition.x, sphere->mPosition.y, 0.0f);
#endif

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
		Work(work.spheres, work.spheres + work.numSpheres, work.blockers, work.blockers + work.numBlockers);

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
		const auto numSpheres = gMovingSpheres.size();
		const auto spheresPerSection = numSpheres / (nThreads + 1);

		auto       spheres = gMovingSpheres.data();
		const auto blockers = gBlockingSpheres.data();

		for (uint32_t i = 0; i < nThreads; ++i)
		{
			auto& work = mUpdateSpheresWorkers[i].second;
			work.numSpheres = spheresPerSection;
			work.spheres = spheres;
			work.blockers = blockers;
			work.numBlockers = gBlockingSpheres.size();

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

		const auto blockersEnd = blockers + gBlockingSpheres.size() - 1;

		// do the remaining work
		Work(spheres, spheres + spheresPerSection, blockers, blockersEnd);

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
		Work(&gMovingSpheres[0], &gMovingSpheres[gMovingSpheres.size() - 1], &gBlockingSpheres[0], &gBlockingSpheres[gBlockingSpheres.size() - 1]);
	}
}


bool GameLoop()
{
	UpdateSpheres();

#ifdef _VISUALIZATION_ON

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

		auto begin = chrono::steady_clock::now();

#ifdef _VISUALIZATION_ON
		// The main game loop, repeat until engine is stopped
		while (myEngine->IsRunning())
#else
		//for (int i = 0; i < 10000; ++i)
		//while(1)
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

#ifdef _VISUALIZATION_ON

			workTime = myEngine->Timer();
#endif

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
