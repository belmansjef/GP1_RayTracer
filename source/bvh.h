#pragma once

#include "DataTypes.h"

namespace dae
{
	struct BVHNode
	{
		Vector3 aabbMin, aabbMax;
		uint8_t leftChild, rightChild;
		bool isLeaf;
		uint8_t firstPrim, primCount;
	};

	class BVH
	{
	public:
		BVH() = default;
		BVH(TriangleMesh* mesh);
		void Build();
		void Intersect(const Ray& ray, HitRecord& hitRecord);

	private:
		void Subdivide(uint8_t nodeIdx);
		void UpdateNodeBounds();
	};
}
