namespace Tmpl8 {
	//static const int N = 12000;
	static const int N = 12582;
	// static const int N = 64;
	//static const int N = 1024;
	static const int BINS = 8;

	__declspec(align(64)) class Ray
	{
	public:
		Ray() = default;
		Ray(const float3 origin, const float3 direction, const float distance = 1e34f, const int idx = -1)
		{
			O = origin, D = direction, t = distance;
			// calculate reciprocal ray direction for triangles and AABBs
			rD = float3(1 / D.x, 1 / D.y, 1 / D.z);
		}
		union { struct { float3 O; float d0; }; __m128 O4; };
		union { struct { float3 D; float d1; }; __m128 D4; };
		union { struct { float3 rD; float d2; }; __m128 rD4; };
		float t = 1e34f;
		int intersect_TRI_count = 0;
		int intersect_AABB_count = 0;
		int traversal_count = 0;
	};

	class Tri {
	public:
		void IntersectTri(Ray& ray)
		{
			ray.intersect_TRI_count++;
			const float3 edge1 = vertex1 - vertex0;
			const float3 edge2 = vertex2 - vertex0;
			const float3 h = cross(ray.D, edge2);
			const float a = dot(edge1, h);
			if (a > -0.0001f && a < 0.0001f) return; // ray parallel to triangle
			const float f = 1 / a;
			const float3 s = ray.O - vertex0;
			const float u = f * dot(s, h);
			if (u < 0 || u > 1) return;
			const float3 q = cross(s, edge1);
			const float v = f * dot(ray.D, q);
			if (v < 0 || u + v > 1) return;
			const float t = f * dot(edge2, q);
			if (t > 0.0001f) ray.t = min(ray.t, t);
		}

		void ComputeAABB() {
			aabbMin = float3(1e30f);
			aabbMax = float3(-1e30f);
			aabbMin = fminf(aabbMin, vertex0);
			aabbMin = fminf(aabbMin, vertex1);
			aabbMin = fminf(aabbMin, vertex2);
			aabbMax = fmaxf(aabbMax, vertex0);
			aabbMax = fmaxf(aabbMax, vertex1);
			aabbMax = fmaxf(aabbMax, vertex2);
		}

		Tri() = default;
		Tri(float3 v0, float3 v1, float3 v2, float3 c) : vertex0(v0), vertex1(v1), vertex2(v2), centroid(c) {}
		float3 vertex0, vertex1, vertex2, centroid, aabbMin, aabbMax;
	};


	class BVHScene {
	public:
		string BVH_MODE = "NO_BASIC";
		struct BVHNode
		{
			union { struct { float3 aabbMin; uint leftFirst; }; __m128 aabbMin4; };
			union { struct { float3 aabbMax; uint triCount; }; __m128 aabbMax4; };
			bool isLeaf() { return triCount > 0; }
		};
		struct aabb
		{
			float3 bmin = 1e30f, bmax = -1e30f;
			void grow(float3 p) { 
				bmin = fminf(bmin, p); 
				bmax = fmaxf(bmax, p); 
			}
			void grow(aabb& b) { 
				if (b.bmin.x != 1e30f) { 
					grow(b.bmin); 
					grow(b.bmax);
				}
			}
			float area()
			{
				float3 e = bmax - bmin; // box extent
				return e.x * e.y + e.y * e.z + e.z * e.x;
			}
		};
		struct Bin { aabb bounds; int triCount = 0; };

		BVHScene() = default;

		void RenderUnityMesh() {
			FILE* file = fopen("../assets/unity.tri", "r");
			for (int t = 0; t < N; t++) {
				fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
					&tri[t].vertex0.x, &tri[t].vertex0.y, &tri[t].vertex0.z,
					&tri[t].vertex1.x, &tri[t].vertex1.y, &tri[t].vertex1.z,
					&tri[t].vertex2.x, &tri[t].vertex2.y, &tri[t].vertex2.z);
				tri[t].centroid = tri[t].vertex0 + tri[t].vertex1 + tri[t].vertex2 * 0.3333f;
			}
		}

		inline float IntersectAABB(Ray& ray, const float3 bmin, const float3 bmax)
		{
			ray.intersect_AABB_count++;
			float tx1 = (bmin.x - ray.O.x) * ray.rD.x, tx2 = (bmax.x - ray.O.x) * ray.rD.x;
			float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
			float ty1 = (bmin.y - ray.O.y) * ray.rD.y, ty2 = (bmax.y - ray.O.y) * ray.rD.y;
			tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
			float tz1 = (bmin.z - ray.O.z) * ray.rD.z, tz2 = (bmax.z - ray.O.z) * ray.rD.z;
			tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
			if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin; else return 1e30f;
		}

		float IntersectAABB_SSE( Ray& ray, const __m128& bmin4, const __m128& bmax4)
		{
			ray.intersect_AABB_count += 1;
			static __m128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
			__m128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmin4, mask4), ray.O4), ray.rD4);
			__m128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmax4, mask4), ray.O4), ray.rD4);
			__m128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
			float tmax = min(vmax4.m128_f32[0], min(vmax4.m128_f32[1], vmax4.m128_f32[2]));
			float tmin = max(vmin4.m128_f32[0], max(vmin4.m128_f32[1], vmin4.m128_f32[2]));
			if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin; else return 1e30f;
		}

		void FindNearestTri(Ray& ray) {
			for (int i = 0; i < N; i++) {
				tri[i].IntersectTri(ray);
			}
		}

		void IntersectBVH(Ray& ray)
		{
			BVHNode* node = &bvhNode[rootNodeIdx], * stack[64];
			uint stackPtr = 0;
			while (1)
			{
				ray.traversal_count++;
				if (node->isLeaf())
				{
					for (uint i = 0; i < node->triCount; i++)
						tri[triIdx[node->leftFirst + i]].IntersectTri(ray);
					if (stackPtr == 0) break; else node = stack[--stackPtr];
					continue;
				}
				BVHNode* child1 = &bvhNode[node->leftFirst];
				BVHNode* child2 = &bvhNode[node->leftFirst + 1];
#ifdef USE_SSE
				
				float dist1 = IntersectAABB_SSE(ray, child1->aabbMin4, child1->aabbMax4);
				float dist2 = IntersectAABB_SSE(ray, child2->aabbMin4, child2->aabbMax4);
#else
				float dist1 = IntersectAABB(ray, child1->aabbMin, child1->aabbMax);
				float dist2 = IntersectAABB(ray, child2->aabbMin, child2->aabbMax);
#endif
				if (dist1 > dist2) { swap(dist1, dist2); swap(child1, child2); }
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
		}

		void BuildBVH()
		{
			bvhNode = (BVHNode*)_aligned_malloc(sizeof(BVHNode) * N * 2, 64);
			for (int i = 0; i < N; i++) triIdx[i] = i;

			// assign all triangles to root node
			BVHNode& root = bvhNode[rootNodeIdx];
			root.leftFirst = 0;
			root.triCount = N;
			UpdateNodeBounds(rootNodeIdx);

			Timer t;
			if (BVH_MODE == "BASIC")
				Subdivide_BASIC(rootNodeIdx);
			else
				Subdivide(rootNodeIdx);
			printf("BVH (%i nodes) constructed in %.2fms.\n", nodesUsed, t.elapsed() * 1000);
		}

		void UpdateNodeBounds(uint nodeIdx)
		{
			BVHNode& node = bvhNode[nodeIdx];
			node.aabbMin = float3(1e30f);
			node.aabbMax = float3(-1e30f);
			for (uint first = node.leftFirst, i = 0; i < node.triCount; i++)
			{
				uint leafTriIdx = triIdx[first + i];
				Tri& leafTri = tri[leafTriIdx];
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex0);
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex1);
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex2);
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex0);
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex1);
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex2);
			}
		}

		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos)
		{
			float bestCost = 1e30f;
			for (int a = 0; a < 3; a++)
			{
				float boundsMin = 1e30f, boundsMax = -1e30f;
				for (uint i = 0; i < node.triCount; i++)
				{
					Tri& triangle = tri[triIdx[node.leftFirst + i]];
					boundsMin = min(boundsMin, triangle.centroid[a]);
					boundsMax = max(boundsMax, triangle.centroid[a]);
				}
				if (boundsMin == boundsMax) continue;
				// populate the bins
				Bin bin[BINS];
				float scale = BINS / (boundsMax - boundsMin);
				for (uint i = 0; i < node.triCount; i++)
				{
					Tri& triangle = tri[triIdx[node.leftFirst + i]];
					int binIdx = min(BINS - 1, (int)((triangle.centroid[a] - boundsMin) * scale));
					bin[binIdx].triCount++;
					bin[binIdx].bounds.grow(triangle.vertex0);
					bin[binIdx].bounds.grow(triangle.vertex1);
					bin[binIdx].bounds.grow(triangle.vertex2);
				}
				// gather data for the 7 planes between the 8 bins
				float leftArea[BINS - 1], rightArea[BINS - 1];
				int leftCount[BINS - 1], rightCount[BINS - 1];
				aabb leftBox, rightBox;
				int leftSum = 0, rightSum = 0;
				for (int i = 0; i < BINS - 1; i++)
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
				// calculate SAH cost for the 7 planes
				scale = (boundsMax - boundsMin) / BINS;
				for (int i = 0; i < BINS - 1; i++)
				{
					float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
					if (planeCost < bestCost)
						axis = a, splitPos = boundsMin + scale * (i + 1), bestCost = planeCost;
				}
			}
			return bestCost;
		}

		float CalculateNodeCost(BVHNode& node)
		{
			float3 e = node.aabbMax - node.aabbMin; // extent of the node
			float surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x;
			return node.triCount * surfaceArea;
		}

		void Subdivide(uint nodeIdx)
		{
			// terminate recursion
			BVHNode& node = bvhNode[nodeIdx];
			// determine split axis using SAH
			int axis;
			float splitPos;
			float splitCost = FindBestSplitPlane(node, axis, splitPos);
			float nosplitCost = CalculateNodeCost(node);
			if (splitCost >= nosplitCost) return;
			// in-place partition
			int i = node.leftFirst;
			int j = i + node.triCount - 1;
			while (i <= j)
			{
				if (tri[triIdx[i]].centroid[axis] < splitPos)
					i++;
				else
					swap(triIdx[i], triIdx[j--]);
			}
			// abort split if one of the sides is empty
			int leftCount = i - node.leftFirst;
			if (leftCount == 0 || leftCount == node.triCount) return;
			// create child nodes
			int leftChildIdx = nodesUsed++;
			int rightChildIdx = nodesUsed++;
			bvhNode[leftChildIdx].leftFirst = node.leftFirst;
			bvhNode[leftChildIdx].triCount = leftCount;
			bvhNode[rightChildIdx].leftFirst = i;
			bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
			node.leftFirst = leftChildIdx;
			node.triCount = 0;
			UpdateNodeBounds(leftChildIdx);
			UpdateNodeBounds(rightChildIdx);
			// recurse
			Subdivide(leftChildIdx);
			Subdivide(rightChildIdx);
		}

		void Subdivide_BASIC(uint nodeIdx)
		{
			// terminate recursion
			BVHNode& node = bvhNode[nodeIdx];
			if (node.triCount <= 2) return;
			// determine split axis and position
			float3 extent = node.aabbMax - node.aabbMin;
			int axis = 0;
			if (extent.y > extent.x) axis = 1;
			if (extent.z > extent[axis]) axis = 2;
			float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;
			// in-place partition
			int i = node.leftFirst;
			int j = i + node.triCount - 1;
			while (i <= j)
			{
				if (tri[triIdx[i]].centroid[axis] < splitPos)
					i++;
				else
					swap(triIdx[i], triIdx[j--]);
			}
			// abort split if one of the sides is empty
			int leftCount = i - node.leftFirst;
			if (leftCount == 0 || leftCount == node.triCount) return;
			// create child nodes
			int leftChildIdx = nodesUsed++;
			int rightChildIdx = nodesUsed++;
			bvhNode[leftChildIdx].leftFirst = node.leftFirst;
			bvhNode[leftChildIdx].triCount = leftCount;
			bvhNode[rightChildIdx].leftFirst = i;
			bvhNode[rightChildIdx].triCount = node.triCount - leftCount;
			node.leftFirst = leftChildIdx;
			node.triCount = 0;
			UpdateNodeBounds(leftChildIdx);
			UpdateNodeBounds(rightChildIdx);
			// recurse
			Subdivide_BASIC(leftChildIdx);
			Subdivide_BASIC(rightChildIdx);
		}


		Tri tri[N];
		uint triIdx[N];
		BVHNode* bvhNode = 0;
		uint rootNodeIdx = 0, nodesUsed = 2;
	};

}