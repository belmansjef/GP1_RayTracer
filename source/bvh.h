#pragma once
#include "DataTypes.h"

namespace dae
{
	struct BVHNode
	{
		Vector3 aabbMin, aabbMax;
		int leftChild, rightChild;
		uint8_t firstPrim, primCount;

		bool IsLeaf()
		{
			return primCount > 0;
		}
	};

	class BVH
	{
	public:
		BVH() = default;
		BVH(TriangleMesh* mesh);
		void Build();
		bool Intersect(const Ray& ray, HitRecord& hitRecord, uint8_t nodeIdx);
		std::vector<BVHNode> m_Nodes;
		uint8_t rootNodeIdx = 0, nodesUsed = 1;
		TriangleMesh* m_MeshPtr = nullptr;

	private:
		void Subdivide(uint8_t nodeIdx);
		void UpdateNodeBounds(uint8_t nodeIdx);
		uint8_t m_TriCount;
	};
}
