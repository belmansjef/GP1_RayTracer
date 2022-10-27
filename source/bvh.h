#pragma once
#include "DataTypes.h"

namespace dae
{
	struct BVHNode
	{
		Vector3 aabbMin, aabbMax;
		uint8_t leftNode, firstTriIdx, triCount;

		bool IsLeaf() { return triCount > 0; }
	};

	class BVH
	{
	public:
		BVH() = default;
		BVH(TriangleMesh* mesh);

		void Build();
		void IntersectBVH(const Ray& ray, HitRecord& hitRecord, uint8_t nodeIdx);
		void UpdateAllNodeBounds(uint8_t nodeIdx);

		std::vector<BVHNode> m_Nodes;
		std::vector<uint8_t> m_TriIdx;
		uint8_t rootNodeIdx = 0, nodesUsed = 1;
		TriangleMesh* m_pMesh = nullptr;

	private:
		void Subdivide(uint8_t nodeIdx);
		void UpdateNodeBounds(uint8_t nodeIdx);
		uint8_t m_TriCount;
	};
}
