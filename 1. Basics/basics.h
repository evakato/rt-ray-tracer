namespace Tmpl8 {
	static const int BINS = 8;

	// intersection record, carefully tuned to be 16 bytes in size
	struct Intersection
	{
		float t;		// intersection distance along ray
		float u, v;		// barycentric coordinates of the intersection
		uint instPrim;	// instance index (12 bit) and primitive index (20 bit)
	};

	__declspec(align(64)) class Ray
	{
	public:
		Ray() = default;
		Ray(const float3 origin, const float3 direction, const float distance = 1e34f)
		{
			O = origin, D = direction, t = distance;
			// calculate reciprocal ray direction for triangles and AABBs
			rD = float3(1 / D.x, 1 / D.y, 1 / D.z);
		}
		union { struct { float3 O; float d0; }; __m128 O4; };
		union { struct { float3 D; float d1; }; __m128 D4; };
		union { struct { float3 rD; float d2; }; __m128 rD4; };
		Intersection hit;
		float t = 1e34f;
	};

	class Tri {
	public:
		void IntersectTri(Ray& ray)
		{
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

		void IntersectTri(Ray& ray, const uint instPrim)
		{
			// Moeller-Trumbore ray/triangle intersection algorithm, see:
			// en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
			const float3 edge1 = vertex1 - vertex0;
			const float3 edge2 = vertex2 - vertex0;
			const float3 h = cross(ray.D, edge2);
			const float a = dot(edge1, h);
			if (fabs(a) < 0.00001f) return; // ray parallel to triangle
			const float f = 1 / a;
			const float3 s = ray.O - vertex0;
			const float u = f * dot(s, h);
			if (u < 0 || u > 1) return;
			const float3 q = cross(s, edge1);
			const float v = f * dot(ray.D, q);
			if (v < 0 || u + v > 1) return;
			const float t = f * dot(edge2, q);
			if (t > 0.0001f && t < ray.hit.t)
				ray.hit.t = t, ray.hit.u = u,
				ray.hit.v = v, ray.hit.instPrim = instPrim;
		}

		bool IntersectTriGrid(Ray& ray)
		{
			const float3 edge1 = vertex1 - vertex0;
			const float3 edge2 = vertex2 - vertex0;
			const float3 h = cross(ray.D, edge2);
			const float a = dot(edge1, h);
			if (a > -0.0001f && a < 0.0001f) return false; // ray parallel to triangle
			const float f = 1 / a;
			const float3 s = ray.O - vertex0;
			const float u = f * dot(s, h);
			if (u < 0 || u > 1) return false;
			const float3 q = cross(s, edge1);
			const float v = f * dot(ray.D, q);
			if (v < 0 || u + v > 1) return false;
			const float t = f * dot(edge2, q);
			if (t > 0.0001f) { ray.t = min(ray.t, t); return true; }
			return false;
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

	// additional triangle data, for texturing and shading
	struct TriEx { float2 uv0, uv1, uv2; float3 N0, N1, N2; };

	struct BVHNode
	{
		union { struct { float3 aabbMin; uint leftFirst; }; __m128 aabbMin4; };
		union { struct { float3 aabbMax; uint triCount; }; __m128 aabbMax4; };
		bool isLeaf() { return triCount > 0; }
		float CalculateNodeCost()
		{
			float3 e = aabbMax - aabbMin; // extent of the node
			return (e.x * e.y + e.y * e.z + e.z * e.x) * triCount;
		}
	};
	struct aabb
	{
		float3 bmin = 1e30f, bmax = -1e30f;
		void grow(float3 p) { bmin = fminf(bmin, p); bmax = fmaxf(bmax, p); }
		void grow(aabb& b) { if (b.bmin.x != 1e30f) { grow(b.bmin); grow(b.bmax); } }
		float area()
		{
			float3 e = bmax - bmin; // box extent
			return e.x * e.y + e.y * e.z + e.z * e.x;
		}
	};
	struct Bin { aabb bounds; int triCount = 0; };

	struct TLASNode
	{
		float3 aabbMin;
		uint leftRight; // 2x16 bits
		float3 aabbMax;
		uint BLAS;
		bool isLeaf() { return leftRight == 0; }
	};

}

