#include "bvh.h"
#include "Utils.h"

#include <chrono>
#include <ctime>

using namespace std::chrono;

namespace dae
{
	float IntersectAABB(const Ray& ray, const Vector3& bmin, const Vector3& bmax)
	{
		float tx1 = (bmin.x - ray.origin.x) * ray.rD.x;
		float tx2 = (bmax.x - ray.origin.x) * ray.rD.x;

		float tmin = std::min(tx1, tx2);
		float tmax = std::max(tx1, tx2);

		float ty1 = (bmin.y - ray.origin.y) * ray.rD.y;
		float ty2 = (bmax.y - ray.origin.y) * ray.rD.y;

		tmin = std::max(tmin, std::min(ty1, ty2));
		tmax = std::min(tmax, std::max(ty1, ty2));

		float tz1 = (bmin.z - ray.origin.z) * ray.rD.z;
		float tz2 = (bmax.z - ray.origin.z) * ray.rD.z;

		tmin = std::max(tmin, std::min(tz1, tz2));
		tmax = std::min(tmax, std::max(tz1, tz2));

		if (tmax > 0 && tmax >= tmin) return tmin;
		else return 1e30f;
	}

#ifdef USE_SSE
	float IntersectAABB_SSE(const Ray& ray, const __m128 bmin4, const __m128 bmax4)
	{
		static __m128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
		__m128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmin4, mask4), ray.O4), ray.rD4);
		__m128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmax4, mask4), ray.O4), ray.rD4);
		__m128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
		float tmax = std::min(vmax4.m128_f32[0], std::min(vmax4.m128_f32[1], vmax4.m128_f32[2]));
		float tmin = std::max(vmin4.m128_f32[0], std::max(vmin4.m128_f32[1], vmin4.m128_f32[2]));
		if (tmax >= tmin && tmax > 0) return tmin; else return 1e30f;
	}
#endif // USE_SSE

	BVH::BVH(TriangleMesh& mesh)
		: m_TriCount{ (mesh.normals.size()) }
	{
		std::cout << "Triangle count: " << m_TriCount << "\n";

		m_Nodes = new BVHNode[m_TriCount * 2 - 1];
		m_TriIdx = new uint64_t[m_TriCount];
		m_Triangles = &mesh.triangles[0];
		for (int i = 0; i < m_TriCount; i++) m_TriIdx[i] = i;

		auto start = high_resolution_clock::now();

		Build();

		auto end = high_resolution_clock::now();
		auto elapsedSeconds = duration_cast<microseconds>(end - start);
		std::cout << "Finished building BVH.\tElapsed time: " << elapsedSeconds.count() << "ms\n";
	}

	BVH::~BVH()
	{
		delete[] m_Nodes;
		delete[] m_TriIdx;

		m_Nodes = nullptr;
		m_TriIdx = nullptr;
	}

	void BVH::Build()
	{
		BVHNode& root = m_Nodes[0];
		root.leftFirst = 0;
		root.triCount = m_TriCount;
		UpdateNodeBounds(0);
		Subdivide(0);
	}

	void BVH::Intersect(Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord, uint64_t nodeIdx)
	{
		Ray originalRay = ray;
		ray.origin = invTransform.TransformPoint(ray.origin);
		ray.direction = invTransform.TransformVector(ray.direction);
		ray.rD = Vector3::Reciprocal(ray.direction);
		
		
		
		BVHNode* node = &m_Nodes[0], *stack[64];
		uint8_t stackPtr = 0;
		while (1)
		{
			if (node->IsLeaf())
			{
				for (uint64_t i = 0; i < node->triCount; i++)
				{
					if (ignoreHitRecord)
					{
						if (GeometryUtils::HitTest_Triangle(m_Triangles[m_TriIdx[node->leftFirst + i]], ray, hitRecord, ignoreHitRecord)) return;
					}
					else
					{
						GeometryUtils::HitTest_Triangle(m_Triangles[m_TriIdx[node->leftFirst + i]], ray, hitRecord);
					}
				}
				if (stackPtr == 0) break; else node = stack[--stackPtr];

				continue;
			} // IsLeaf

			BVHNode* child1 = &m_Nodes[node->leftFirst];
			BVHNode* child2 = &m_Nodes[node->leftFirst + 1];
#ifdef USE_SSE
			float dist1 = IntersectAABB_SSE(ray, child1->aabbMin4, child1->aabbMax4);
			float dist2 = IntersectAABB_SSE(ray, child2->aabbMin4, child2->aabbMax4);
#else
			float dist1 = IntersectAABB(ray, child1->aabbMin, child1->aabbMax);
			float dist2 = IntersectAABB(ray, child2->aabbMin, child2->aabbMax);
#endif
			
			if (dist1 > dist2) { std::swap(dist1, dist2); std::swap(child1, child2); }
			if (dist1 == 1e30f)
			{
				if (stackPtr == 0) break; else node = stack[--stackPtr];
			}
			else
			{
				node = child1;
				if (dist2 != 1e30f) stack[stackPtr++] = child2;
			}
		}
		
		ray = originalRay;
	}

	void BVH::Refit()
	{
		for (int i = m_NodesUsed - 1; i >= 0; i--) if (i != 1)
		{
			BVHNode& node = m_Nodes[i];
			if (node.IsLeaf())
			{
				// leaf node: adjust bounds to contained triangles
				UpdateNodeBounds(i);
				continue;
			}
			// interior node: adjust bounds to child node bounds
			BVHNode& leftChild = m_Nodes[node.leftFirst];
			BVHNode& rightChild = m_Nodes[node.leftFirst + 1];
			node.aabbMin = Vector3::Min(leftChild.aabbMin, rightChild.aabbMin);
			node.aabbMax = Vector3::Max(leftChild.aabbMax, rightChild.aabbMax);
		}
	}

	void BVH::SetTransform(Matrix& transform)
	{
		invTransform = transform.Inverted();

		// calculate world-space bounds using the new matrix
		Vector3 bmin = m_Nodes[0].aabbMin, bmax = m_Nodes[0].aabbMax;
		bounds = aabb();
		for (int i = 0; i < 8; i++)
		{
			Vector3 position{ i & 1 ? bmax.x : bmin.x, i & 2 ? bmax.y : bmin.y, i & 4 ? bmax.z : bmin.z };
			bounds.grow(transform.TransformPoint(position));
		}	
	}

	void BVH::Subdivide(uint64_t nodeIdx)
	{
		BVHNode& node = m_Nodes[nodeIdx];

		uint8_t axis;
		float splitPos;
		float splitCost = FindBestSplitPlane(node, axis, splitPos);
		float nosplitCost = node.CalculateNodeCost();
		if (splitCost >= nosplitCost) return;
		
		// Rearange tri's
		uint64_t i = node.leftFirst;
		uint64_t j = i + node.triCount - 1;
		while (i <= j)
		{
			if (m_Triangles[m_TriIdx[i]].centroid[axis] < splitPos)
				i++;
			else
				std::swap(m_TriIdx[i], m_TriIdx[j--]);
		}

		uint64_t leftCount = i - node.leftFirst;
		if (leftCount == 0 || leftCount == node.triCount) return;

		uint64_t leftChildIdx = m_NodesUsed++;
		uint64_t rightChildIdx = m_NodesUsed++;
		m_Nodes[leftChildIdx].leftFirst = node.leftFirst;
		m_Nodes[leftChildIdx].triCount = leftCount;
		m_Nodes[rightChildIdx].leftFirst = i;
		m_Nodes[rightChildIdx].triCount = node.triCount - leftCount;
		node.leftFirst = leftChildIdx;
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
		for (uint64_t first = node.leftFirst, i = 0; i < node.triCount; i++)
		{
			uint64_t leafTriIdx = m_TriIdx[first + i];
			Triangle& leafTri = m_Triangles[leafTriIdx];
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v0);
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v1);
			node.aabbMin = Vector3::Min(node.aabbMin, leafTri.v2);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v0);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v1);
			node.aabbMax = Vector3::Max(node.aabbMax, leafTri.v2);
		}
	}

	float BVH::FindBestSplitPlane(BVHNode& node, uint8_t& axis, float& splitPos)
	{
		float bestCost = 1e30f;
		for (uint8_t a = 0; a < 3; a++)
		{
			float boundsMin = 1e30f, boundsMax = -1e30f;
			for (uint64_t i = 0; i < node.triCount; i++)
			{
				Triangle& triangle = m_Triangles[m_TriIdx[node.leftFirst + i]];
				boundsMin = std::min(boundsMin, triangle.centroid[a]);
				boundsMax = std::max(boundsMax, triangle.centroid[a]);
			}
			if (boundsMin == boundsMax) continue;
			// Populate bins
			struct Bin { aabb bounds; uint64_t triCount = 0; } bin[BINS];
			float scale = BINS / (boundsMax - boundsMin);
			for (uint64_t i = 0; i < node.triCount; i++)
			{
				Triangle& triangle = m_Triangles[m_TriIdx[node.leftFirst + i]];
				uint64_t binIdx = std::min(BINS - 1,
					(int)((triangle.centroid[a] - boundsMin) * scale));
				bin[binIdx].triCount++;
				bin[binIdx].bounds.grow(triangle.v0);
				bin[binIdx].bounds.grow(triangle.v1);
				bin[binIdx].bounds.grow(triangle.v2);
			}
			// Gather data for the planes between the bins
			float leftArea[BINS - 1], rightArea[BINS - 1];
			uint64_t leftCount[BINS - 1], rightCount[BINS - 1];
			aabb leftBox, rightBox;
			uint64_t leftSum = 0, rightSum = 0;
			for (uint64_t i = 0; i < BINS - 1; i++)
			{
				leftSum += bin[i].triCount;
				leftCount[i] = leftSum;
				leftBox.grow(bin[i].bounds);
				leftArea[i] = leftBox.area();
				rightSum += bin[BINS - 1 - i].triCount;
				rightCount[BINS - 2 - i] = rightSum;
				rightBox.grow(bin[BINS - 1 - i].bounds);
				rightArea[BINS - 2 - i] = rightBox.area();
			}
			// Calculate SAH cost for the planes
			scale = (boundsMax - boundsMin) / BINS;
			for (uint64_t i = 1; i < BINS - 1; i++)
			{
				float planeCost =
					leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
				if (planeCost < bestCost)
					axis = a, splitPos = boundsMin + scale * (i + 1),
					bestCost = planeCost;
			}
		}
		return bestCost;
	}
}