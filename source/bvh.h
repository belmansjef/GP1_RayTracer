#pragma once
#include "DataTypes.h"

#define BINS 3

namespace dae
{
	struct BVHNode
	{
		BVHNode(){};
		
		union { struct { Vector3 aabbMin; uint64_t firstTriIdx; }; __m128 aabbMin4; };
		union { struct { Vector3 aabbMax; uint64_t triCount; }; __m128 aabbMax4; };
		union { struct { Vector3 dummy1; uint64_t leftNode; }; __m128 dummy; };

		bool IsLeaf() const { return triCount > 0; }
		float CalculateNodeCost()
		{
			Vector3 e = aabbMax - aabbMin;
			return (e.x * e.y + e.y * e.z + e.z * e.x) * triCount;
		}
	};

	struct aabb
	{
		Vector3 bmin = { 1e30f, 1e30f, 1e30f }, bmax = {-1e30f, -1e30f, -1e30f};
		void grow(Vector3 p) { bmin = Vector3::Min(bmin, p), bmax = Vector3::Max(bmax, p); }
		void grow(aabb& b) { if (b.bmin.x != 1e30f) { grow(b.bmin); grow(b.bmax); } }
		float area()
		{
			Vector3 e = bmax = bmin;
			return e.x * e.y + e.y * e.z + e.z * e.x;
		}
	};

	struct Bin { aabb bounds; uint64_t triCount = 0; };

	class BVH
	{
	public:
		BVH() = default;
		BVH(TriangleMesh& mesh);

		void Build();
		void Intersect(const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false, uint64_t nodeIdx = 0);
		void Refit();

	private:
		void Subdivide(uint64_t nodeIdx);
		void UpdateNodeBounds(uint64_t nodeIdx);
		float FindBestSplitPlane(BVHNode& node, uint8_t& axis, float& splitPos);
		float CalculateNodeCost(BVHNode& node);

		BVHNode* m_Nodes = 0;
		Triangle* m_Triangles = 0;
		uint64_t* m_TriIdx = 0;
		uint64_t m_NodesUsed = 1, m_TriCount;
	};
}
