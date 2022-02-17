// Assignment.cpp: A program using the TL-Engine

#include <TL-Engine.h>	// TL-Engine include file and namespace
#include <Eigen/Eigen>
#include <random>


using namespace tle;
using namespace Eigen;

I3DEngine* myEngine;

std::random_device rd;

struct SSphere
{
	IModel* mModel;
	TFloat32 mRadius = 1.f;
	Vector2f mVelocity;
	Vector2f mPosition;
	Vector3f mColour;
	uint8_t mHealth = 100;
	char mName[];
};


std::vector<SSphere> gSpheres;

constexpr uint32_t KNumOfSpheres = 100;
constexpr TFloat32 KRangeSpawn = 1000.f;
constexpr TFloat32 KRangeVelocity = 5.f;

Vector2f KWallBoundsMax = Vector2f(KRangeSpawn,KRangeSpawn);
Vector2f KWallBoundsMin = -KWallBoundsMax;

ICamera* myCamera;

bool GameLoop();

bool SceneSetup();

void main()
{
	// Create a 3D engine (using TLX engine here) and open a window for it
	myEngine = New3DEngine(kTLX);
	myEngine->StartWindowed();

	// Add default folder for meshes and other media
	myEngine->AddMediaFolder("C:\\ProgramData\\TL-Engine\\Media");

	/**** Set up your scene here ****/

	SceneSetup();

	auto frameTime = 0.0f;
	const auto font = myEngine->DefaultFont();

	// The main game loop, repeat until engine is stopped
	while (myEngine->IsRunning())
	{
		// Draw the scene
		myEngine->DrawScene();

		/**** Update your scene each frame here ****/

		if (!GameLoop()) break;


		static float t = 1.f;

		if (t < 0.f)
		{
			frameTime = myEngine->Timer();
			t = 1.f;
		}
		else t -= myEngine->Timer();

		font->Draw("ms: " + std::to_string(frameTime), 10, 10);
		font->Draw("FPS: " + std::to_string(1.0f / frameTime), 10, 20);
	}

	// Delete the 3D engine now we are finished with it
	myEngine->Delete();
}


bool SceneSetup()
{

	myCamera = myEngine->CreateCamera(kFPS);

	std::mt19937_64 mt(rd());
	const std::uniform_real_distribution<float> spawnDist(-KRangeSpawn, KRangeSpawn);
	const std::uniform_real_distribution<float> velocityDist(-KRangeVelocity, KRangeVelocity);

	gSpheres.reserve(KNumOfSpheres);

	const auto sphereMesh = myEngine->LoadMesh("Sphere.x");

	for (auto i = 0u; i < KNumOfSpheres; ++i)
	{
		SSphere s;

		s.mPosition = Vector2f(spawnDist(mt), spawnDist(mt));

		s.mModel = sphereMesh->CreateModel();

		if (i < KNumOfSpheres / 2)
		{
			s.mVelocity= Vector2f(velocityDist(mt), velocityDist(mt));
		}

		gSpheres.push_back(s);
	}
	
	return true;
}


void CheckForCollisions()
{
	for (auto& sphere : gSpheres)
	{
		auto& v = sphere.mVelocity;
		auto p = Vector2f(sphere.mModel->GetX(),sphere.mModel->GetY());

		for(auto& other : gSpheres)
		{
			auto v2 = other.mVelocity;
			auto p2 = Vector2f(other.mModel->GetX(), other.mModel->GetY());
			

		}
	}
}


Vector2f Reflect(Vector2f& vec, Vector2f& surfaceNormal)
{
	surfaceNormal.normalize();
	return vec - 2.0f * surfaceNormal * (surfaceNormal.dot(vec));
}


void UpdateSpheres()
{
	for (auto& sphere : gSpheres)
	{
		if (sphere.mPosition.x() > KWallBoundsMax.x() ||
			sphere.mPosition.x() < KWallBoundsMin.x())
		{
			auto surfaceNormal = Vector2f(0.f, 1.f);
			sphere.mVelocity = Reflect(sphere.mVelocity, surfaceNormal);
		}
		else if (sphere.mPosition.y() > KWallBoundsMax.y() ||
			sphere.mPosition.y() < KWallBoundsMin.y())
		{
			auto surfaceNormal = Vector2f(1.f, 0.f);
			sphere.mVelocity = Reflect(sphere.mVelocity, surfaceNormal);
		}

		sphere.mPosition += sphere.mVelocity;
		sphere.mModel->SetPosition(sphere.mPosition.x(), sphere.mPosition.y(), 0.0f);
	}
}


bool GameLoop()
{

	CheckForCollisions();

	UpdateSpheres();

	if (myEngine->KeyHit(Key_Escape)) return false;

	return true;
}