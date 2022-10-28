#include "bvh.h"
#include "Utils.h"

namespace dae
{
	inline bool IntersectAABB(const Ray& ray, const Vector3& bmin, const Vector3& bmax)
	{
		float tx1 = (bmin.x - ray.origin.x) / ray.direction.x;
		float tx2 = (bmax.x - ray.origin.x) / ray.direction.x;

		float tmin = std::min(tx1, tx2);
		float tmax = std::max(tx1, tx2);

		float ty1 = (bmin.y - ray.origin.y) / ray.direction.y;
		float ty2 = (bmax.y - ray.origin.y) / ray.direction.y;

		tmin = std::max(tmin, std::min(ty1, ty2));
		tmax = std::min(tmax, std::max(ty1, ty2));

		float tz1 = (bmin.z - ray.origin.z) / ray.direction.z;
		float tz2 = (bmax.z - ray.origin.z) / ray.direction.z;

		tmin = std::max(tmin, std::min(tz1, tz2));
		tmax = std::min(tmax, std::max(tz1, tz2));

		return tmax > 0 && tmax >= tmin;
	}

	BVH::BVH(TriangleMesh* mesh)
		: m_pMesh{ mesh }
		, m_TriCount{ (uint64_t)(mesh->triangles.size()) }
	{
		Build();
	}

	void BVH::Build()
	{
		m_TriIdx.reserve(m_TriCount);
		m_Nodes.reserve(m_TriCount * 2 - 1);
		for (int i = 0; i < m_TriCount; i++)
		{
			m_TriIdx.emplace_back(i);
		}
		for (int i = 0; i < m_TriCount * 2 - 1; i++)
		{
			m_Nodes.emplace_back(BVHNode());
		}
		
		BVHNode& root = m_Nodes[rootNodeIdx];
		root.leftNode = root.firstTriIdx = 0;
		root.triCount = m_TriCount;
		UpdateNodeBounds(rootNodeIdx);
		Subdivide(rootNodeIdx);
	}

	void BVH::IntersectBVH(const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord, uint64_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];
		if (!IntersectAABB(ray, node.aabbMin, node.aabbMax)) return;
		if (node.IsLeaf())
		{
			for (uint8_t i = 0; i < node.triCount; i++)
			{
				if (ignoreHitRecord)
				{
					if (GeometryUtils::HitTest_Triangle(m_pMesh->triangles[m_TriIdx[node.firstTriIdx + i]], ray, hitRecord, ignoreHitRecord)) return;
				}
				else
				{
					GeometryUtils::HitTest_Triangle(m_pMesh->triangles[m_TriIdx[node.firstTriIdx + i]], ray, hitRecord);
				}
			}
		}
		else
		{
			IntersectBVH(ray, hitRecord, ignoreHitRecord, node.leftNode);
			IntersectBVH(ray, hitRecord, ignoreHitRecord, node.leftNode + 1);
		}
	}

	void BVH::UpdateAllNodeBounds(uint64_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];
		UpdateNodeBounds(nodeIdx);

		if (node.IsLeaf()) return;

		UpdateAllNodeBounds(node.leftNode);
		UpdateAllNodeBounds(node.leftNode + 1);
	}

	void BVH::Subdivide(uint64_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];

		Vector3 e = node.aabbMax - node.aabbMin;
		float parentArea = e.x * e.y + e.y * e.z + e.z * e.x;
		float parentCost = node.triCount * parentArea;

		// if (node.triCount < 2) return;

		int bestAxis = -1;
		float bestPos = 0.f, bestCost = 1e30f;
		for (uint8_t axis = 0; axis < 3; axis++) for(uint64_t i = 0; i < node.triCount; i++)
		{
			Triangle& triangle = m_pMesh->triangles[m_TriIdx[node.leftNode + i]];
			float candidatePos = triangle.centroid[axis];
			float cost = EvaluateSAH(node, axis, candidatePos);
			if (cost < bestCost)
				bestPos = candidatePos, bestAxis = axis, bestCost = cost;
		}

		if (bestCost >= parentCost) return;

		uint8_t axis = bestAxis;
		float splitPos = bestPos;

		/*Vector3 extent = node.aabbMax - node.aabbMin;
		int axis = 0;
		if (extent.y > extent.x) axis = 1;
		if (extent.z > extent[axis]) axis = 2;
		float splitPos = (node.aabbMin[axis] + extent[axis]) * 0.5f;*/
		
		uint64_t i = node.firstTriIdx;
		uint64_t j = i + node.triCount - 1;
		while (i <= j)
		{
			if (m_pMesh->triangles[m_TriIdx[i]].centroid[axis] < splitPos)
				i++;
			else
				std::swap(m_TriIdx[i], m_TriIdx[j--]);
		}

		uint64_t leftCount = i - node.firstTriIdx;
		if (leftCount == 0 || leftCount == node.triCount) return;

		uint64_t leftChildIdx = nodesUsed++;
		uint64_t rightChildIdx = nodesUsed++;
		node.leftNode = leftChildIdx;
		m_Nodes[leftChildIdx].firstTriIdx = node.firstTriIdx;
		m_Nodes[leftChildIdx].triCount = leftCount;
		m_Nodes[rightChildIdx].firstTriIdx = i;
		m_Nodes[rightChildIdx].triCount = node.triCount - leftCount;
		node.triCount = 0;

		UpdateNodeBounds(leftChildIdx);
		UpdateNodeBounds(rightChildIdx);

		Subdivide(leftChildIdx);
		Subdivide(rightChildIdx);
	}
	void BVH::UpdateNodeBounds(uint64_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];
		node.aabbMin = Vector3({ INFINITY, INFINITY, INFINITY });
		node.aabbMax = Vector3({ -INFINITY, -INFINITY, -INFINITY });
		for (uint64_t first = node.firstTriIdx, i = 0; i < node.triCount; i++)
		{
			uint64_t leafTriIdx = m_TriIdx[first + i];
			Triangle& leafTri = m_pMesh->triangles[leafTriIdx];
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v0);
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v1);
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v2);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v0);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v1);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v2);
		}
	}
	float BVH::EvaluateSAH(BVHNode& node, uint8_t axis, float pos)
	{
		aabb leftBox, rightBox;
		int leftCount = 0, rightCount = 0;
		for (uint64_t i = 0; i < node.triCount; i++)
		{
			Triangle& triangle = m_pMesh->triangles[m_TriIdx[node.leftNode + i]];
			if (triangle.centroid[axis] < pos)
			{
				leftCount++;
				leftBox.grow(triangle.v0);
				leftBox.grow(triangle.v1);
				leftBox.grow(triangle.v2);
			}
			else
			{
				rightCount++;
				rightBox.grow(triangle.v0);
				rightBox.grow(triangle.v1);
				rightBox.grow(triangle.v2);
			}
		}
		float cost = leftCount * leftBox.area() + rightCount * rightBox.area();
		return cost > 0 ? cost : 1e30f;
	}
}
