namespace Tmpl8 {

	class KDTreeScene {
	public:

		struct aabb
		{
			float3 bmin = 1e30f, bmax = -1e30f;
			float area()
			{
				float3 e = bmax - bmin; // box extent
				return e.x * e.y + e.y * e.z + e.z * e.x;
			}
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
		}; 

		struct KDNode
		{
			float3 aabbMin, aabbMax;
			uint leftNode;
			// rightNode is just leftNode + 1
			vector<int> tris;
		};

		struct Bin { aabb bounds; int triCount = 0; };
		
		KDTreeScene() = default;

		void Start() {
			RenderUnityMesh();
			//RenderTriangles();
			BuildKDTree();
		}

		void RenderUnityMesh() {
			FILE* file = fopen("../assets/unity.tri", "r");
			for (int t = 0; t < N; t++) {
				fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
					&tri[t].vertex0.x, &tri[t].vertex0.y, &tri[t].vertex0.z,
					&tri[t].vertex1.x, &tri[t].vertex1.y, &tri[t].vertex1.z,
					&tri[t].vertex2.x, &tri[t].vertex2.y, &tri[t].vertex2.z);
				tri[t].ComputeAABB();
			}
		}

		void RenderTriangles() {
			for (int i = 0; i < N; i++)
			{
				float3 r0 = float3(RandomFloat(), RandomFloat(), RandomFloat());
				float3 r1 = float3(RandomFloat(), RandomFloat(), RandomFloat());
				float3 r2 = float3(RandomFloat(), RandomFloat(), RandomFloat());
				tri[i].vertex0 = r0 * 9 - float3(5);
				tri[i].vertex1 = tri[i].vertex0 + r1, tri[i].vertex2 = tri[i].vertex0 + r2;
				tri[i].ComputeAABB();
			}
		}

		inline float IntersectAABB(const Ray& ray, const float3 bmin, const float3 bmax)
		{
			float tx1 = (bmin.x - ray.O.x) * ray.rD.x, tx2 = (bmax.x - ray.O.x) * ray.rD.x;
			float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
			float ty1 = (bmin.y - ray.O.y) * ray.rD.y, ty2 = (bmax.y - ray.O.y) * ray.rD.y;
			tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
			float tz1 = (bmin.z - ray.O.z) * ray.rD.z, tz2 = (bmax.z - ray.O.z) * ray.rD.z;
			tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
			if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin; else return 1e30f;
		}

		void FindNearestTri(Ray& ray) {
			for (int i = 0; i < N; i++) {
				tri[i].IntersectTri(ray);
			}
		}

		void IntersectKD(Ray& ray)
		{
			KDNode* node = &kdNode[rootNodeIdx], * stack[64];
			uint stackPtr = 0;
			while (1)
			{
				if (node->tris.size() > 0)
				{
					for (uint i = 0; i < node->tris.size(); i++)
						tri[node->tris[i]].IntersectTri(ray);
					if (stackPtr == 0) break; else node = stack[--stackPtr];
					continue;
				}
				KDNode* child1 = &kdNode[node->leftNode];
				KDNode* child2 = &kdNode[node->leftNode + 1];

				float dist1 = IntersectAABB(ray, child1->aabbMin, child1->aabbMax);
				float dist2 = IntersectAABB(ray, child2->aabbMin, child2->aabbMax);

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

		void BuildKDTree() {
			kdNode = (KDNode*)_aligned_malloc(sizeof(kdNode) * N * sizeof(int) * 4, 64);

			KDNode& root = kdNode[rootNodeIdx];
			root.leftNode = 0;
			vector<int> tris(N);
			for (int i = 0; i < N; i++)
				tris[i] = i;
			root.tris = tris;
			UpdateNodeBounds(rootNodeIdx);
			printf("%f %f %f %f %f %f\n", root.aabbMin[0], root.aabbMin[1], root.aabbMin[2], root.aabbMax[0], root.aabbMax[1], root.aabbMax[2]);

			Timer t;
			Subdivide(rootNodeIdx, 1);
			printf("KD (%i nodes) constructed in %.2fms.\n", nodesUsed, t.elapsed() * 1000);
		}

		void UpdateNodeBounds(uint nodeIdx)
		{
			KDNode& node = kdNode[nodeIdx];
			node.aabbMin = float3(1e30f);
			node.aabbMax = float3(-1e30f);
			for (int i = 0; i < node.tris.size(); i++)
			{
				Tri& leafTri = tri[node.tris[i]];
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex0),
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex1),
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex2),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex0),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex1),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex2);
			}
		}

		float FindBestSplitPlane(KDNode& node, int& axis, float& splitPos)
		{
			float bestCost = 1e30f;
			for (int a = 0; a < 3; a++)
			{
				float boundsMin = node.aabbMin[a], boundsMax = node.aabbMax[a];
				if (boundsMin == boundsMax) continue;
				// populate the bins
				Bin leftBin[BINS];
				Bin rightBin[BINS];
				//printf("%f %f\n",  boundsMin, boundsMax);
				float scale = BINS / (boundsMax - boundsMin);
				for (uint i = 0; i < node.tris.size(); i++)
				{
					Tri& triangle = tri[node.tris[i]];
					if (triangle.aabbMin[a] >= boundsMin) {
						int leftBinIdx = min(BINS - 1, (int)((triangle.aabbMin[a] - boundsMin) * scale));
						leftBin[leftBinIdx].triCount++;
						leftBin[leftBinIdx].bounds.grow(triangle.vertex0);
						leftBin[leftBinIdx].bounds.grow(triangle.vertex1);
						leftBin[leftBinIdx].bounds.grow(triangle.vertex2);
					}
					if (boundsMax >= triangle.aabbMax[a]) {
						int rightBinIdx = BINS - 1 - max(0, (int)((boundsMax - triangle.aabbMax[a]) * scale));
						rightBin[rightBinIdx].triCount++;
						rightBin[rightBinIdx].bounds.grow(triangle.vertex0);
						rightBin[rightBinIdx].bounds.grow(triangle.vertex1);
						rightBin[rightBinIdx].bounds.grow(triangle.vertex2);
					}

					//printf("%d %d %f %f\n", leftBinIdx, rightBinIdx, triangle.aabbMin[a], triangle.aabbMax[a]);
				}
				//for (int i = 0; i < BINS; i++)
					//printf("%d left bin tri count %d right bin tri count %d \n", i, leftBin[i].triCount, rightBin[i].triCount);
				// gather data for the 7 planes between the 8 bins
				float leftArea[BINS - 1], rightArea[BINS - 1];
				int leftCount[BINS - 1], rightCount[BINS - 1];
				aabb leftBox, rightBox;
				int leftSum = 0, rightSum = 0;
				scale = (boundsMax - boundsMin) / BINS;
				for (int i = 0; i < BINS - 1; i++)
				{
					leftSum += leftBin[i].triCount;
					leftCount[i] = leftSum;
					leftBox.grow(leftBin[i].bounds);
					leftBox.bmax[a] = boundsMin + (i + 1) * scale;
					leftArea[i] = leftBox.area();
				}
				for (int i = BINS - 2; i >= 0; i--) {
					rightSum += rightBin[i+1].triCount;
					rightCount[i] = rightSum;
					rightBox.grow(rightBin[i + 1].bounds);
					rightBox.bmin[a] = boundsMin + scale * (i + 1);
					rightArea[i] = rightBox.area();
				}
				// calculate SAH cost for the 7 planes
				for (int i = 0; i < BINS - 1; i++)
				{
					float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
					//printf("axis %d plane: %d best cost: %f plane cost %f left count %d right count %d\n", a, i, bestCost, planeCost, leftCount[i], rightCount[i]);
					if (planeCost < bestCost)
						axis = a, splitPos = boundsMin + scale * (i + 1), bestCost = planeCost;
				}
			}
			return bestCost;
		}

		float CalculateNodeCost(KDNode& node)
		{
			float3 e = node.aabbMax - node.aabbMin; // extent of the node
			float surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x;
			return node.tris.size() * surfaceArea;
		}

#if 0
		aabb CalculateBox(uint firstTri, uint triCount) {
			aabb box;
			for (uint first = firstTri, i = 0; i < triCount; i++) {
				uint leafTriIdx = triIdx[first + i];
				Tri& leafTri = tri[leafTriIdx];
				box.bmin = fminf(box.bmin, leafTri.vertex0);
				box.bmin = fminf(box.bmin, leafTri.vertex1);
				box.bmin = fminf(box.bmin, leafTri.vertex2);
				box.bmax = fmaxf(box.bmax, leafTri.vertex0);
				box.bmax = fmaxf(box.bmax, leafTri.vertex1);
				box.bmax = fmaxf(box.bmax, leafTri.vertex2);
			}
			return box;
		}
#endif
		void PrintTris(KDNode &node) {
			for (int i = 0; i < N; i++)
				printf("%f %f %f %f %f %f\n", tri[node.tris[i]].aabbMin[0],  tri[node.tris[i]].aabbMin[1], tri[node.tris[i]].aabbMin[2], tri[node.tris[i]].aabbMax[0], tri[node.tris[i]].aabbMax[1],tri[node.tris[i]].aabbMax[2]);
		}

		void Subdivide(uint nodeIdx, int maxDepth)
		{
			if (maxDepth >= 13 ) return;
			KDNode& node = kdNode[nodeIdx];

			int axis;
			float splitPos;
			float splitCost = FindBestSplitPlane(node, axis, splitPos);
			float nosplitCost = CalculateNodeCost(node);
			//printf("split: %f nosplit: %f pos: %f axis: %d\n", splitCost, nosplitCost, splitPos, axis);

			if (splitCost >= nosplitCost) return;

			vector<int> leftTris = {};
			vector<int> rightTris = {};
			for (int i = 0; i < node.tris.size(); i++) {
				if (tri[node.tris[i]].aabbMin[axis] <= splitPos) {
					leftTris.push_back(node.tris[i]);
				}
				if (tri[node.tris[i]].aabbMax[axis] >= splitPos) {
					rightTris.push_back(node.tris[i]);
				}
			}
			//PrintTris(node);
			//printf("left: %d right: %d\n", leftTris.size(), rightTris.size());

			// abort split if one of the sides is empty
			if (leftTris.size() == 0 || rightTris.size() == 0) return;

			// create child nodes
			int leftChildIdx = nodesUsed++;
			int rightChildIdx = nodesUsed++;
			node.leftNode = leftChildIdx;
			kdNode[leftChildIdx].tris = leftTris;
			kdNode[rightChildIdx].tris = rightTris;
			node.tris = {};
			UpdateNodeBounds(leftChildIdx);
			UpdateNodeBounds(rightChildIdx);
			kdNode[leftChildIdx].aabbMax[axis] = splitPos;
			kdNode[rightChildIdx].aabbMin[axis] = splitPos;

			// recurse
			Subdivide(leftChildIdx, maxDepth + 1);
			Subdivide(rightChildIdx, maxDepth + 1);
		}

		Tri tri[N];
		KDNode* kdNode = 0;
		uint rootNodeIdx = 0, nodesUsed = 1;
	};
}
