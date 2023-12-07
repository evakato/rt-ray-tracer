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
			uint firstTri;
			uint triCount;
			bool isLeaf() { return triCount > 0; }
			void printNode() {
				printf("left: %d right: %d firstTri: %d triCount: %d \n", leftNode, leftNode == 0 ? 0 : leftNode + 1, firstTri, triCount);
				printf("\taabb: %f %f %f %f %f %f\n", aabbMin[0], aabbMin[1], aabbMin[2], aabbMax[0], aabbMax[1], aabbMax[2]);
			}
		};
		
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

		/*
		void IntersectKD(Ray& ray, const uint nodeIdx)
		{
			KDNode& node = kdNode[nodeIdx];
			if (IntersectAABB(ray, node.aabbMin, node.aabbMax) == 1e30f)
				return;
			if (node.isLeaf())
			{
				for (uint i = 0; i < node.triCount; i++)
					tri[triIdx[node.firstTri + i]].IntersectTri(ray);
			}
			else
			{
				IntersectKD(ray, node.leftNode);
				IntersectKD(ray, node.leftNode + 1);
			}
		}*/

		void IntersectKD(Ray& ray)
		{
			KDNode* node = &kdNode[rootNodeIdx], * stack[64];
			uint stackPtr = 0;
			while (1)
			{
				if (node->isLeaf())
				{
					for (uint i = 0; i < node->triCount; i++)
						tri[triIdx[node->firstTri + i]].IntersectTri(ray);
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
			kdNode = (KDNode*)_aligned_malloc(sizeof(kdNode) * N * 4, 64);
			for (int i = 0; i < N; i++) triIdx[i] = i;

			KDNode& root = kdNode[rootNodeIdx];
			root.leftNode = 0;
			root.firstTri = 0;
			root.triCount = N;
			UpdateNodeBounds(rootNodeIdx);
			printf("root\n");
			root.printNode();

			Timer t;
			Subdivide(rootNodeIdx, 0, 1);
			printf("KD (%i nodes) constructed in %.2fms.\n", nodesUsed, t.elapsed() * 1000);

#if 0
			for (int i = 1000; i < nodesUsed; i++) {
				printf("%d ---- ", i);
				kdNode[i].printNode();
			}
#endif
		}

		void UpdateNodeBounds(uint nodeIdx)
		{
			KDNode& node = kdNode[nodeIdx];
			node.aabbMin = float3(1e30f);
			node.aabbMax = float3(-1e30f);
			for (uint first = node.firstTri, i = 0; i < node.triCount; i++)
			{
				uint leafTriIdx = triIdx[first + i];
				Tri& leafTri = tri[leafTriIdx];
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex0),
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex1),
				node.aabbMin = fminf(node.aabbMin, leafTri.vertex2),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex0),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex1),
				node.aabbMax = fmaxf(node.aabbMax, leafTri.vertex2);
			}
		}

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

		void PrintTris(int start, int end, int axis) {
			for (int ind = start; ind < end; ind++) {
				//printf("%f %f %f %f %f %f\n", tri[triIdx[ind]].aabbMin[0], tri[triIdx[ind]].aabbMin[1], tri[triIdx[ind]].aabbMin[2], tri[triIdx[ind]].aabbMax[0], tri[triIdx[ind]].aabbMax[1], tri[triIdx[ind]].aabbMax[2]);
				printf("%d: %f %f \n", ind, tri[triIdx[ind]].aabbMin[axis], tri[triIdx[ind]].aabbMax[axis]);
			}
		}

		float EvaluateSAH(KDNode& node, int axis, float pos)
		{
			// determine triangle counts and bounds for this split candidate
			aabb leftBox, rightBox;
			int leftCount = 0, rightCount = 0;
			for (uint i = 0; i < node.triCount; i++)
			{
				Tri& triangle = tri[triIdx[node.firstTri + i]];
				if (triangle.aabbMin[axis] <= pos)
				{
					leftCount++;
					leftBox.grow(triangle.vertex0);
					leftBox.grow(triangle.vertex1);
					leftBox.grow(triangle.vertex2);
				}
				if (triangle.aabbMax[axis] >= pos)
				{
					rightCount++;
					rightBox.grow(triangle.vertex0);
					rightBox.grow(triangle.vertex1);
					rightBox.grow(triangle.vertex2);
				}
			}
			leftBox.bmax[axis] = pos;
			rightBox.bmin[axis] = pos;
			float cost = leftCount * leftBox.area() + rightCount * rightBox.area();
			return cost > 0 ? cost : 1e30f;
		}

		void Subdivide(uint nodeIdx, int axis, int maxDepth)
		{
			//if (nodesUsed >= N - 1) return;
			KDNode& node = kdNode[nodeIdx];
			if (maxDepth >= 5 ) return;

			// determine split axis and position
			float3 extent = node.aabbMax - node.aabbMin;
			float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;

			// abort split if cost sucks 
#if 0
			float cost = EvaluateSAH(node, axis, splitPos);
			float3 e = node.aabbMax - node.aabbMin; // extent of parent
			float parentArea = e.x * e.y + e.y * e.z + e.z * e.x;
			float parentCost = node.triCount * parentArea;
			if (cost >= parentCost) return;
#endif

			// in-place partition
			int i = node.firstTri;
			int j = i + node.triCount - 1;
			while (i <= j)
			{
				if (tri[triIdx[i]].aabbMin[axis] < splitPos)
					i++;
				else
					swap(triIdx[i], triIdx[j--]);
			}
			int k = node.firstTri;
			int ov = i - 1; // triangles overlapping with split plane
			while (k <= ov) {
				if (tri[triIdx[k]].aabbMax[axis] < splitPos)
					k++;
				else
					swap(triIdx[k], triIdx[ov--]);
			}

			// abort split if one of the sides is empty
			int leftCount = i - node.firstTri;
			int rightCount = node.triCount - (k - node.firstTri);
			printf("axis: %d splitpos: %f i: %d ov:%d k: %d\n ---- left first: %d left count: %d right first: %d right count: %d\n", axis, splitPos, i, ov, k, node.firstTri, leftCount, k, rightCount);
			PrintTris(node.firstTri, node.firstTri + node.triCount, axis);
			if (leftCount == 0 || leftCount == node.triCount || rightCount == 0) return;
			/*
			aabb leftBox = CalculateBox(node.firstTri, leftCount);
			leftBox.bmax[axis] = splitPos;
			float leftCost = leftCount * leftBox.area();
			aabb rightBox = CalculateBox(k, rightCount);
			rightBox.bmin[axis] = splitPos;
			float rightCost = rightCount * rightBox.area();
			float combinedCost = leftCost + rightCost > 0 ? leftCost + rightCost : 1e30f;
			*/
			//printf("left: %d %f %f right: %d %f %f parent: %d %f %f", leftCount, leftBox.area(), leftCost, rightCount, rightBox.area(), rightCost, node.triCount, parentArea, parentCost);
			//if (combinedCost >= parentCost) return;

			// create child nodes
			int leftChildIdx = nodesUsed++;
			int rightChildIdx = nodesUsed++;
			node.leftNode = leftChildIdx;
			kdNode[leftChildIdx].firstTri = node.firstTri;
			kdNode[leftChildIdx].triCount = leftCount;
			kdNode[rightChildIdx].firstTri = k;
			kdNode[rightChildIdx].triCount = rightCount;
			node.triCount = 0;
			UpdateNodeBounds(leftChildIdx);
			UpdateNodeBounds(rightChildIdx);
			//kdNode[leftChildIdx].aabbMax = leftBox.bmax;
			//kdNode[leftChildIdx].aabbMin = leftBox.bmin;
			//kdNode[rightChildIdx].aabbMax = rightBox.bmax;
			//kdNode[rightChildIdx].aabbMin = rightBox.bmin;
			kdNode[leftChildIdx].aabbMax[axis] = splitPos;
			kdNode[rightChildIdx].aabbMin[axis] = splitPos;

			/*
			printf("\nleft child first tri: %d, tricount: %d\n", kdNode[leftChildIdx].firstTri, kdNode[leftChildIdx].triCount);
			printf("left aabb: %f %f %f %f %f %f\n", kdNode[leftChildIdx].aabbMin[0], kdNode[leftChildIdx].aabbMin[1], kdNode[leftChildIdx].aabbMin[2], kdNode[leftChildIdx].aabbMax[0], kdNode[leftChildIdx].aabbMax[1], kdNode[leftChildIdx].aabbMax[2]);
			printf("right child first tri: %d, tricount: %d\n", kdNode[rightChildIdx].firstTri, kdNode[rightChildIdx].triCount);
			printf("right aabb: %f %f %f %f %f %f\n", kdNode[rightChildIdx].aabbMin[0], kdNode[rightChildIdx].aabbMin[1], kdNode[rightChildIdx].aabbMin[2], kdNode[rightChildIdx].aabbMax[0], kdNode[rightChildIdx].aabbMax[1], kdNode[rightChildIdx].aabbMax[2]);
			printf("\n");
			*/

			// recurse
			Subdivide(leftChildIdx, (axis + 1) % 3, maxDepth + 1);
			Subdivide(rightChildIdx, (axis + 1) % 3, maxDepth + 1);
		}


		Tri tri[N];
		uint triIdx[N];
		KDNode* kdNode = 0;
		uint rootNodeIdx = 0, nodesUsed = 1;
	};
}
