namespace Tmpl8 {

	class BVHScene {
	public:
		string BVH_MODE = "SAH"; // SAH or BASIC
		BVHNode* bvhNode = 0;

		BVHScene() = default;
		BVHScene(char* triFile, int N);
		BVHScene(class Mesh* triMesh);
		void FindNearestTri(Ray& ray);
		void IntersectBVH(Ray& ray, uint instanceIdx);
		void BuildBVH();
		void Build();
		float CalculateNodeCost(BVHNode& node);

	private:
		Tri* tri = 0;
		uint* triIdx = 0;
		uint rootNodeIdx = 0, nodesUsed = 2;
		uint N;
		class Mesh* mesh = 0;

		void Subdivide(uint nodeIdx);
		void UpdateNodeBounds(uint nodeIdx);
		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos);
	};	

	// instance of a BVH, with transform and world bounds
	class BVHInstance
	{
	public:
		aabb bounds; // in world space

		BVHInstance() = default;
		BVHInstance(BVHScene* blas) : bvh(blas) { SetTransform(mat4()); }
		BVHInstance(BVHScene* blas, uint index) : bvh(blas), idx( index ) { SetTransform(mat4()); }
		void SetTransform(mat4& transform);
		void Intersect(Ray& ray);
	private:
		uint idx;

		BVHScene* bvh = 0;
		mat4 invTransform; // inverse transform
	};

	class TLAS
	{
	public:
		TLAS() = default;
		TLAS(BVHInstance* bvhList, int N);
		void Build();
		void Intersect(Ray& ray);
		int FindBestMatch(int* list, int N, int A);
	private:
		TLASNode* tlasNode = 0;
		BVHInstance* blas = 0;
		uint nodesUsed, blasCount;
	};


}