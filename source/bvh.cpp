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
		: m_MeshPtr { mesh }
	{
		Build();
	}
	void BVH::Build()
	{
		m_TriCount = (m_MeshPtr->indices.size() / 3.f);
		m_Nodes.reserve(m_TriCount * 2.f - 1.f);

		BVHNode& root = m_Nodes[rootNodeIdx];
		root.leftChild = root.rightChild = 0;
		root.firstPrim = 0, root.primCount = m_TriCount;
		UpdateNodeBounds(rootNodeIdx);
		Subdivide(rootNodeIdx);

	}
	bool BVH::Intersect(const Ray& ray, HitRecord& hitRecord, uint8_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];
		if (!IntersectAABB(ray, node.aabbMin, node.aabbMax)) return false;
		if (node.IsLeaf())
		{
			for (uint8_t i = 0; i < node.primCount; i++)
			{
				if (GeometryUtils::HitTest_Triangle(m_MeshPtr->triangles[i], ray, hitRecord)) return true;
			}
			return false;
		}
		else
		{
			Intersect(ray, hitRecord, node.leftChild);
			Intersect(ray, hitRecord, node.rightChild);
		}

		return false;
	}
	void BVH::Subdivide(uint8_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];
		if (node.primCount < 2) return;

		Vector3 extent = node.aabbMax - node.aabbMin;
		int axis = 0;
		if (extent.y > extent.x) axis = 1;
		if (extent.z > extent[axis]) axis = 2;
		float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;
		int i = node.firstPrim;
		int j = i + node.primCount - 1;
		while (i <= j)
		{
			if (m_MeshPtr->triangles[i].centroid[axis] < splitPos)
				i++;
			else
				std::swap(m_MeshPtr->triangles[i], m_MeshPtr->triangles[j--]);
		}

		int leftCount = i - node.firstPrim;
		if (leftCount == 0 || leftCount == node.primCount) return;

		int leftChildIdx = nodesUsed++;
		int rightChildIdx = nodesUsed++;
		node.leftChild = leftChildIdx;
		m_Nodes[leftChildIdx].firstPrim = node.firstPrim;
		m_Nodes[leftChildIdx].primCount = leftCount;
		m_Nodes[rightChildIdx].firstPrim = i;
		m_Nodes[rightChildIdx].primCount = node.primCount - leftCount;
		node.primCount = 0;

		UpdateNodeBounds(leftChildIdx);
		UpdateNodeBounds(rightChildIdx);

		Subdivide(leftChildIdx);
		Subdivide(rightChildIdx);
	}
	void BVH::UpdateNodeBounds(uint8_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];
		node.aabbMin = Vector3({ INFINITY, INFINITY, INFINITY });
		node.aabbMax = Vector3({ -INFINITY, -INFINITY, -INFINITY });
		uint16_t first = node.firstPrim;
		for (uint16_t i = 0; i < node.primCount; i += 3)
		{
			Triangle& leafTri = m_MeshPtr->triangles[first + i];
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v0);
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v1);
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v2);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v0);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v1);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v2);
		}
	}
}
