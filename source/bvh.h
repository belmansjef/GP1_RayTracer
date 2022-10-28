#pragma once
#include "DataTypes.h"

namespace dae
{
	struct BVHNode
	{
		Vector3 aabbMin, aabbMax;
		uint64_t leftNode, firstTriIdx, triCount;

		bool IsLeaf() { return triCount > 0; }
	};

	class BVH
	{
	public:
		BVH() = default;
		BVH(TriangleMesh* mesh);

		void Build();
		void IntersectBVH(const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false, uint64_t nodeIdx = 0);
		void UpdateAllNodeBounds(uint64_t nodeIdx);

		std::vector<BVHNode> m_Nodes;
		std::vector<uint64_t> m_TriIdx;
		uint64_t rootNodeIdx = 0, nodesUsed = 1;
		TriangleMesh* m_pMesh = nullptr;

	private:
		void Subdivide(uint64_t nodeIdx);
		void UpdateNodeBounds(uint64_t nodeIdx);
		uint64_t m_TriCount;
	};
}
