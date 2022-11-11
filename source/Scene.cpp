#include "Scene.h"
#include "Utils.h"
#include "Material.h"

namespace dae {
#pragma region Base Scene
	//Initialize Scene with Default Solid Color Material (RED)
	Scene::Scene():
		m_Materials({ new Material_SolidColor({1,0,0})})
	{
		m_SphereGeometries.reserve(32);
		m_PlaneGeometries.reserve(32);
		m_TriangleMeshGeometries.reserve(32);
		m_Lights.reserve(32);
	}

	Scene::~Scene()
	{
		for(auto& pMaterial : m_Materials)
		{
			delete pMaterial;
			pMaterial = nullptr;
		}

		m_Materials.clear();
		for (auto& bvh : m_pBVH)
		{
			delete bvh;
			bvh = nullptr;
		}
		m_pBVH.clear();
	}

	void dae::Scene::GetClosestHit(Ray& ray, HitRecord& closestHit) const
	{
		for (const Sphere& sphere: m_SphereGeometries)
		{
			GeometryUtils::HitTest_Sphere(sphere, ray, closestHit);
		}

		for (const Plane& plane : m_PlaneGeometries)
		{
			GeometryUtils::HitTest_Plane(plane, ray, closestHit);
		}

#ifdef USE_BVH

		for (const auto& bvh : m_pBVH)
			bvh->Intersect(ray, closestHit, false);
#else
		for (const TriangleMesh& triangleMesh : m_TriangleMeshGeometries)
		{
			GeometryUtils::HitTest_TriangleMesh(triangleMesh, ray, closestHit);
		}
#endif // USE_BVH
	}

	bool Scene::DoesHit(Ray& ray) const
	{
	
		for (const Sphere& sphere: m_SphereGeometries)
		{
			if (GeometryUtils::HitTest_Sphere(sphere, ray))
				return true;
		}

		for (const Plane& plane : m_PlaneGeometries)
		{
			if (GeometryUtils::HitTest_Plane(plane, ray))
				return true;
		}

#ifdef USE_BVH
		HitRecord temp{};
		for (const auto& bvh : m_pBVH)
		{
			bvh->Intersect(ray, temp, true);
			if (temp.didHit)
				return true;
		}
#else
		for (const TriangleMesh& triangleMesh : m_TriangleMeshGeometries)
		{
			if (GeometryUtils::HitTest_TriangleMesh(triangleMesh, ray))
				return true;
		}
#endif // USE_BVH

		return false;
	}

#pragma region Scene Helpers
	Sphere* Scene::AddSphere(const Vector3& origin, float radius, unsigned char materialIndex)
	{
		Sphere s;
		s.origin = origin;
		s.radius = radius;
		s.materialIndex = materialIndex;

		m_SphereGeometries.emplace_back(s);
		return &m_SphereGeometries.back();
	}

	Plane* Scene::AddPlane(const Vector3& origin, const Vector3& normal, unsigned char materialIndex)
	{
		Plane p;
		p.origin = origin;
		p.normal = normal;
		p.materialIndex = materialIndex;

		m_PlaneGeometries.emplace_back(p);
		return &m_PlaneGeometries.back();
	}

	TriangleMesh* Scene::AddTriangleMesh(TriangleCullMode cullMode, unsigned char materialIndex)
	{
		TriangleMesh m{};
		m.cullMode = cullMode;
		m.materialIndex = materialIndex;

		m_TriangleMeshGeometries.emplace_back(m);
		return &m_TriangleMeshGeometries.back();
	}

	Light* Scene::AddPointLight(const Vector3& origin, float intensity, const ColorRGB& color)
	{
		Light l;
		l.origin = origin;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Point;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	Light* Scene::AddDirectionalLight(const Vector3& direction, float intensity, const ColorRGB& color)
	{
		Light l;
		l.direction = direction;
		l.intensity = intensity;
		l.color = color;
		l.type = LightType::Directional;

		m_Lights.emplace_back(l);
		return &m_Lights.back();
	}

	unsigned char Scene::AddMaterial(Material* pMaterial)
	{
		m_Materials.push_back(pMaterial);
		return static_cast<unsigned char>(m_Materials.size() - 1);
	}
#pragma endregion
#pragma endregion

#pragma region SCENE W1
	void Scene_W1::Initialize()
	{
		//default: Material id0 >> SolidColor Material (RED)
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		//Spheres
		AddSphere({ -25.f, 0.f, 100.f }, 50.f, matId_Solid_Red);
		AddSphere({ 25.f, 0.f, 100.f }, 50.f, matId_Solid_Blue);

		//Plane
		AddPlane({ -75.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 75.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0.f, -75.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 75.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 0.f, 125.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);
	}
#pragma endregion

#pragma region SCENE W2
	void Scene_W2::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;
		m_Camera.CalculateFov();

		// default:: Material id0 >> SolidColor Material (RED)
		constexpr unsigned char matId_Solid_Red = 0;
		const unsigned char matId_Solid_Blue = AddMaterial(new Material_SolidColor{ colors::Blue });

		const unsigned char matId_Solid_Yellow = AddMaterial(new Material_SolidColor{ colors::Yellow });
		const unsigned char matId_Solid_Green = AddMaterial(new Material_SolidColor{ colors::Green });
		const unsigned char matId_Solid_Magenta = AddMaterial(new Material_SolidColor{ colors::Magenta });

		// Plane
		AddPlane({ -5.f, 0.f, 0.f }, { 1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 5.f, 0.f, 0.f }, { -1.f, 0.f,0.f }, matId_Solid_Green);
		AddPlane({ 0.f, 0.f, 0.f }, { 0.f, 1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 10.f, 0.f }, { 0.f, -1.f,0.f }, matId_Solid_Yellow);
		AddPlane({ 0.f, 0.f, 10.f }, { 0.f, 0.f,-1.f }, matId_Solid_Magenta);

		// Spheres
		AddSphere({ -1.75f, 1.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ 0.f, 1.f, 0.f }, .75f, matId_Solid_Blue);
		AddSphere({ 1.75f, 1.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ -1.75f, 3.f, 0.f }, .75f, matId_Solid_Blue);
		AddSphere({ 0.f, 3.f, 0.f }, .75f, matId_Solid_Red);
		AddSphere({ 1.75f, 3.f, 0.f }, .75f, matId_Solid_Blue);

		// Light
		AddPointLight({ 0.f, 5.f, -5.f }, 70.f, colors::Cyan);
	}
#pragma endregion

#pragma region SCENE W3
	void Scene_W3::Initialize()
	{
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;
		m_Camera.CalculateFov();

		const auto matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, 1.f));
		const auto matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .6f));
		const auto matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .1f));
		const auto matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, 1.f));
		const auto matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .6f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .1f));

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, .57f, .57f }, 1.f));

		//Plane
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Left

		//Spheres
		AddSphere(Vector3{ -1.75, 1.f, 0.f }, .75f, matCT_GrayRoughMetal);
		AddSphere(Vector3{ 0, 1.f, 0.f }, .75f, matCT_GrayMediumMetal);
		AddSphere(Vector3{ 1.75, 1.f, 0.f }, .75f, matCT_GraySmoothMetal);
		AddSphere(Vector3{ -1.75, 3.f, 0.f }, .75f, matCT_GrayRoughPlastic);
		AddSphere(Vector3{ 0, 3.f, 0.f }, .75f, matCT_GrayMediumPlastic);
		AddSphere(Vector3{ 1.75, 3.f, 0.f }, .75f, matCT_GraySmoothPlastic);

		//Light
		AddPointLight(Vector3{ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, 0.61f, .45f });
		AddPointLight(Vector3{ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, 0.8f, .45f });
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, 0.47f, .68f });
	}
#pragma endregion
	
#pragma region SCENE W4
	void Scene_W4::Initialize()
	{
		m_Camera.origin = { 0.f, 1.f, -5.f };
		m_Camera.fovAngle = 45.f;
		m_Camera.CalculateFov();

		//Materials
		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ 0.49f, 0.57f, 0.57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//Plane
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Left

		//Triangle Mesh

		pMesh = AddTriangleMesh(TriangleCullMode::NoCulling, matLambert_White);
		Utils::ParseOBJ("Resources/simple_cube.obj",
			pMesh->positions,
			pMesh->normals,
			pMesh->indices);

		pMesh->Scale({ 0.7f, 0.7f, 0.7f });
		pMesh->Translate({ 0.f, 1.5f, 0.f });
#ifndef USE_BVH
		pMesh->UpdateAABB();
#endif
		pMesh->UpdateTransforms();

#ifdef USE_BVH
		m_pBVH.emplace_back(new BVH(*pMesh));
#endif

		//Light
		AddPointLight(Vector3{ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, 0.61f, .45f }); //backLight
		AddPointLight(Vector3{ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, 0.8f, .45f }); //front light left
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, 0.47f, .68f });
	}

	void Scene_W4::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		pMesh->RotateY(PI_DIV_2 * pTimer->GetTotal());
		pMesh->UpdateTransforms();
#ifdef USE_BVH
		for (const auto& bvh : m_pBVH)
		{
			bvh->Refit();
		}
#endif
	}
#pragma endregion

#pragma region SCENE W4_ReferenceScene
	void Scene_W4_ReferenceScene::Initialize()
	{
		sceneName = "Reference Scene";
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;
		m_Camera.CalculateFov();

		const auto matCT_GrayRoughMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, 1.f));
		const auto matCT_GrayMediumMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .6f));
		const auto matCT_GraySmoothMetal = AddMaterial(new Material_CookTorrence({ .972f, .960f, .915f }, 1.f, .1f));
		const auto matCT_GrayRoughPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, 1.f));
		const auto matCT_GrayMediumPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .6f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, .1f));

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, .57f, .57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//Plane
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Left

		//Spheres
		AddSphere(Vector3{ -1.75, 1.f, 0.f }, .75f, matCT_GrayRoughMetal);
		AddSphere(Vector3{ 0, 1.f, 0.f }, .75f, matCT_GrayMediumMetal);
		AddSphere(Vector3{ 1.75, 1.f, 0.f }, .75f, matCT_GraySmoothMetal);
		AddSphere(Vector3{ -1.75, 3.f, 0.f }, .75f, matCT_GrayRoughPlastic);
		AddSphere(Vector3{ 0, 3.f, 0.f }, .75f, matCT_GrayMediumPlastic);
		AddSphere(Vector3{ 1.75, 3.f, 0.f }, .75f, matCT_GraySmoothPlastic);

		const Triangle baseTriangle = { Vector3(-.75f, 1.5f, 0.f), Vector3(0.75f, 0.f, 0.f), Vector3(-.75f, 0.f, 0.f)};

		m_Triangles[0] = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		m_Triangles[0]->AppendTriangle(baseTriangle, true);
		m_Triangles[0]->Translate({ -1.75f, 4.5f, 0.f });
#ifndef USE_BVH
		m_Triangles[0]->UpdateAABB();
#endif // !USE_BVH
		m_Triangles[0]->UpdateTransforms();

		m_Triangles[1] = AddTriangleMesh(TriangleCullMode::FrontFaceCulling, matLambert_White);
		m_Triangles[1]->AppendTriangle(baseTriangle, true);
		m_Triangles[1]->Translate({ 0.f, 4.5f, 0.f });
#ifndef USE_BVH
		m_Triangles[1]->UpdateAABB();
#endif // !USE_BVH
		m_Triangles[1]->UpdateTransforms();

		m_Triangles[2] = AddTriangleMesh(TriangleCullMode::NoCulling, matLambert_White);
		m_Triangles[2]->AppendTriangle(baseTriangle, true);
		m_Triangles[2]->Translate({ 1.75f, 4.5f, 0.f });
#ifndef USE_BVH
		m_Triangles[2]->UpdateAABB();
#endif // !USE_BVH
		m_Triangles[2]->UpdateTransforms();

#ifdef USE_BVH
		for (const auto& m : m_Triangles)
		{
			m_pBVH.emplace_back(new BVH(*m));
		}
#endif // USE_BVH


		//Light
		AddPointLight(Vector3{ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, 0.61f, .45f });
		AddPointLight(Vector3{ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, 0.8f, .45f });
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, 0.47f, .68f });
	}

	void Scene_W4_ReferenceScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		const auto yawAngle = (cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2;
		for (const auto mesh : m_Triangles)
		{
			mesh->RotateY(yawAngle);
			mesh->UpdateTransforms();
		}

#ifdef USE_BVH
		for (const auto& bvh : m_pBVH)
		{
			bvh->Refit();
		}
#endif
	}
#pragma endregion

#pragma region SCENE W4_BunnyScene
	void Scene_W4_BunnyScene::Initialize()
	{
		sceneName = "Bunny Scene";
		m_Camera.origin = { 0.f, 3.f, -9.f };
		m_Camera.fovAngle = 45.f;
		m_Camera.CalculateFov();

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, .57f, .57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));

		//Plane
		AddPlane(Vector3{ 0.f, 0.f, 10.f }, Vector3{ 0.f, 0.f, -1.f }, matLambert_GrayBlue);; //Back
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue);; //Bottom
		AddPlane(Vector3{ 0.f, 10.f, 0.f }, Vector3{ 0.f, -1.f, 0.f }, matLambert_GrayBlue);; //Top
		AddPlane(Vector3{ 5.f, 0.f, 0.f }, Vector3{ -1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Right
		AddPlane(Vector3{ -5.f, 0.f, 0.f }, Vector3{ 1.f, 0.f, 0.f }, matLambert_GrayBlue);; //Left

		m_Bunny = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/lowpoly_bunny2.obj",
			m_Bunny->positions,
			m_Bunny->normals,
			m_Bunny->indices);

		m_Bunny->Scale({ 2.f, 2.f, 2.f });

#ifndef USE_BVH
		m_Bunny->UpdateAABB();
#endif // !USE_BVH
		m_Bunny->UpdateTransforms();

#ifdef USE_BVH
		m_pBVH.push_back(new BVH(*m_Bunny));
#endif // USE_BVH

		//Light
		AddPointLight(Vector3{ 0.f, 5.f, 5.f }, 50.f, ColorRGB{ 1.f, 0.61f, .45f });
		AddPointLight(Vector3{ -2.5f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, 0.8f, .45f });
		AddPointLight(Vector3{ 2.5f, 2.5f, -5.f }, 50.f, ColorRGB{ .34f, 0.47f, .68f });
	}
	void Scene_W4_BunnyScene::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);

		const auto yawAngle = (cos(pTimer->GetTotal()) + 1.f) / 2.f * PI_2;

		/*Matrix translation = Matrix::CreateRotationY(yawAngle);
		m_pBVH[0]->SetTransform(translation);*/

		m_Bunny->RotateY(yawAngle);
		m_Bunny->UpdateTransforms();
#ifdef USE_BVH
		for (auto& bvh : m_pBVH)
		{
			bvh->Refit();
		}
 #endif
	}
#pragma endregion

#pragma region Formula_1
	void Scene_Formula1::Initialize()
	{
		sceneName = "Formula 1 Scene";
		m_Camera.origin = { 0.f, 2.f, -15.f };
		m_Camera.fovAngle = 65.f;
		m_Camera.CalculateFov();

		const auto matLambert_GrayBlue = AddMaterial(new Material_Lambert({ .49f, .57f, .57f }, 1.f));
		const auto matLambert_White = AddMaterial(new Material_Lambert(colors::White, 1.f));
		const auto matCT_GraySmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, 0.1f));
		const auto matCT_GraySemiSmoothPlastic = AddMaterial(new Material_CookTorrence({ .75f, .75f, .75f }, 0.f, 0.5f));

		//Plane
		AddPlane(Vector3{ 0.f, 0.f, 0.f }, Vector3{ 0.f, 1.f, 0.f }, matLambert_GrayBlue);; //Bottom

		m_F1Car = AddTriangleMesh(TriangleCullMode::BackFaceCulling, matLambert_White);
		Utils::ParseOBJ("Resources/F1.obj",
			m_F1Car->positions,
			m_F1Car->normals,
			m_F1Car->indices);

		m_F1Car->RotateY(-45.f * TO_RADIANS);
		m_F1Car->Translate({ 2.0f, 0.f, 0.f });

		AddSphere(Vector3{ -9.f, 3.f, -1.f }, 2.f, matCT_GraySmoothPlastic);
		AddSphere(Vector3{ 9.f, 3.f, -1.f }, 2.f, matCT_GraySemiSmoothPlastic);
#ifndef USE_BVH
		m_F1Car->UpdateAABB();
#endif // !USE_BVH
		m_F1Car->UpdateTransforms();

#ifdef USE_BVH
		m_pBVH.push_back(new BVH(*m_F1Car));
#endif // USE_BVH

		//Light
		AddPointLight(Vector3{ 2.5f, 10.f, -2.f }, 150.f, ColorRGB{ 0.65f, 0.45f, .45f });
		AddPointLight(Vector3{ -13.f, 5.f, -5.f }, 70.f, ColorRGB{ 1.f, 0.8f, .45f });
		AddPointLight(Vector3{ 12.f, 5.f, -3.f }, 50.f, ColorRGB{ .34f, 0.47f, .68f });
	}

	void Scene_Formula1::Update(Timer* pTimer)
	{
		Scene::Update(pTimer);
	}
#pragma endregion
}