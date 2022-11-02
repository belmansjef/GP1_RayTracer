#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

#define Geometric
#define Analytic
#define Moller

namespace dae
{
	namespace GeometryUtils
	{
#pragma region SlabTest
		inline bool SlabTest(const Vector3& minAABB, const Vector3& maxABBB, const Ray& ray)
		{
			float tx1 = (minAABB.x - ray.origin.x) * ray.rD.x;
			float tx2 = (maxABBB.x - ray.origin.x) * ray.rD.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (minAABB.y - ray.origin.y) * ray.rD.y;
			float ty2 = (maxABBB.y - ray.origin.y) * ray.rD.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (minAABB.z - ray.origin.z) * ray.rD.z;
			float tz2 = (maxABBB.z - ray.origin.z) * ray.rD.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}
#pragma endregion
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#if defined(Geometric)
			// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
			const Vector3 L = sphere.origin - ray.origin;
			const float tca = Vector3::Dot(L, ray.direction);
			if (tca < 0) return false;

			const float d2 = Vector3::Dot(L, L) - tca * tca;
			if (d2 > sphere.radius * sphere.radius || d2 < 0) return false;

			const float thc = Sqrt_Intrin(sphere.radius * sphere.radius - d2);
			float t = tca - thc;

			if (t < ray.min)
			{
				t = tca + thc;
				if (t < ray.min) return false;
			}

			if (t > ray.max || hitRecord.t < t) return false;

			if (ignoreHitRecord) return true;

			hitRecord.didHit = true;
			hitRecord.materialIndex = sphere.materialIndex;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
			hitRecord.t = t;
			return true;
#elif defined(Analytic)
			const Vector3 rayToShpere = ray.origin - sphere.origin;
			const float B = 2.f * Vector3::Dot(ray.direction, (rayToShpere));
			const float C = Vector3::Dot(rayToShpere, rayToShpere) - (sphere.radius * sphere.radius);

			const float discriminant{ (B * B) - (4.f * C) };

			if (discriminant < 0) return false;

			const float sqrtDiscriminant = Sqrt_Intrin(discriminant);
			float t = (-B - sqrtDiscriminant) * 0.5f;

			if (t < ray.min)
			{
				t = (-B + sqrtDiscriminant) * 0.5f;
			}

			if (t < ray.min || t > ray.max || hitRecord.t < t) return false;
			
			if (ignoreHitRecord) return true;

			hitRecord.didHit = true;
			hitRecord.materialIndex = sphere.materialIndex;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
			hitRecord.t = t;
			return true;
#endif
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const float t = (Vector3::Dot((plane.origin - ray.origin), plane.normal)) / Vector3::Dot(ray.direction, plane.normal);

			if (t < ray.min || t > ray.max || hitRecord.t < t) return false;

			if (ignoreHitRecord) return true;

			hitRecord.didHit = true;
			hitRecord.materialIndex = plane.materialIndex;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.normal = plane.normal.Normalized();
			hitRecord.t = t;
			return true;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		const float EPSILON = 0.00000000001f;
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#ifdef Moller
			const Vector3 v0v1 = triangle.v1 - triangle.v0;
			const Vector3 v0v2 = triangle.v2 - triangle.v0;
			const Vector3 pvec = Vector3::Cross(ray.direction, v0v2);
			const float det = Vector3::Dot(v0v1, pvec);

			TriangleCullMode cullMode{ triangle.cullMode };
			if (ignoreHitRecord && cullMode != TriangleCullMode::NoCulling)
				cullMode = TriangleCullMode(((int)cullMode + 1) % 2);

			if (det < EPSILON && cullMode == TriangleCullMode::BackFaceCulling) return false;
			if (det > EPSILON && cullMode == TriangleCullMode::FrontFaceCulling) return false;
			if (abs(det) < EPSILON) return false;

			const float invDet = 1 / det;

			const Vector3 tvec = ray.origin - triangle.v0;
			const float u = Vector3::Dot(tvec, pvec) * invDet;
			if (u < 0 || u > 1) return false;

			const Vector3 qvec = Vector3::Cross(tvec, v0v1);
			const float v = Vector3::Dot(ray.direction, qvec) * invDet;
			if (v < 0 || u + v > 1) return false;

			const float t = Vector3::Dot(v0v2, qvec) * invDet;
			if (t < ray.min || t > ray.max || hitRecord.t < t) return false;

			hitRecord.didHit = true;
			if (ignoreHitRecord) return true;

			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.origin = ray.origin + ray.direction * t;
			hitRecord.normal = triangle.normal;
			hitRecord.t = t;
			return true;
#else // Does not work with BVH
			// hitRecord.didHit = false;

			TriangleCullMode cullMode{ triangle.cullMode };
			// Flip cull mode for shadow rays
			if(ignoreHitRecord && cullMode != TriangleCullMode::NoCulling)
				cullMode = TriangleCullMode(((int)cullMode + 1) % 2);

			// If view ray is perpendicular to normal, we can't see the face
			const float rayDotNormal{ Vector3::Dot(ray.direction, triangle.normal) };
			if (
				(rayDotNormal == 0) ||
				(rayDotNormal > 0 && cullMode == TriangleCullMode::BackFaceCulling) ||
				(rayDotNormal < 0 && cullMode == TriangleCullMode::FrontFaceCulling)
				) return false;

			const Vector3 triangleCenter{ (triangle.v0 + triangle.v1 + triangle.v2) / 3.f };
			const Vector3 viewToCenter{ triangleCenter - ray.origin };
			const float t = Vector3::Dot(viewToCenter, triangle.normal) / Vector3::Dot(ray.direction, triangle.normal);

			if (t >= ray.max || t <= ray.min || hitRecord.t < t) return false;

			const Vector3 intersection = ray.origin + (ray.direction * t);

			const Vector3 edgeA{ triangle.v1 - triangle.v0 };
			const Vector3 edgeB{ triangle.v2 - triangle.v1 };
			const Vector3 edgeC{ triangle.v0 - triangle.v2 };
			
			Vector3 pointToIntersect{ intersection - triangle.v0 };
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeA, pointToIntersect)) < 0) return false;

			pointToIntersect =  intersection - triangle.v1;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeB, pointToIntersect)) < 0) return false;

			pointToIntersect = intersection - triangle.v2;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeC, pointToIntersect)) < 0) return false;

			if (ignoreHitRecord) return true;

			hitRecord.didHit = true;
			hitRecord.normal = triangle.normal;
			hitRecord.origin = intersection;
			hitRecord.t = t;
			hitRecord.materialIndex = triangle.materialIndex;
			return true;
#endif // Moller		
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			if (!SlabTest(mesh.transformedMinAABB, mesh.transformedMaxAABB, ray)) return false;

			int normalIndex{ 0 };
			Triangle triangle{};

			for (uint64_t index = 0; index < mesh.indices.size(); index += 3)
			{
				uint32_t i0 = mesh.indices[index];
				uint32_t i1 = mesh.indices[index + 1];
				uint32_t i2 = mesh.indices[index + 2];

				triangle =
				{
					mesh.transformedPositions[i0],
					mesh.transformedPositions[i1],
					mesh.transformedPositions[i2],
					mesh.transformedNormals[normalIndex++]
				};

				triangle.cullMode = mesh.cullMode;
				triangle.materialIndex = mesh.materialIndex;

				if (GeometryUtils::HitTest_Triangle(triangle, ray, hitRecord, ignoreHitRecord))
				{
					if(ignoreHitRecord) return true;
				}
			}

			return hitRecord.didHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3& origin)
		{
			return light.origin - origin;
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			ColorRGB radiance{light.color};

			switch (light.type)
			{
			case LightType::Point:
				radiance *= light.intensity / (light.origin - target).SqrMagnitude();
				break;
			case LightType::Directional:
				radiance *= light.intensity;
				break;
			default:
				break;
			}
			return radiance;
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}