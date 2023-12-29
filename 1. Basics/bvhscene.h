namespace Tmpl8 {
	//static const int N = 12000;
	static const int N = 12582;
	// static const int N = 64;
	//static const int N = 1024;
	static const int BINS = 8;



	class BVHScene {
	public:
		string BVH_MODE = "BASIC";// SAH OR BASIC
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



		inline float IntersectAABB(Ray& ray, const float3 bmin, const float3 bmax);
		float IntersectAABB_SSE(Ray& ray, const __m128& bmin4, const __m128& bmax4);
		void FindNearestTri(Ray& ray);
		void IntersectBVH(Ray& ray);
		void BuildBVH();
		void UpdateNodeBounds(uint nodeIdx);
		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos);
		float CalculateNodeCost(BVHNode& node);
		void Subdivide(uint nodeIdx);
		void Subdivide_BASIC(uint nodeIdx);
		Tri tri[N];
		uint triIdx[N];
		BVHNode* bvhNode = 0;
		uint rootNodeIdx = 0, nodesUsed = 2;
	};

}