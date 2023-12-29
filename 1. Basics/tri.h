#pragma once
namespace Tmpl8 {
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

		bool IntersectTriGrid(Ray& ray)
		{
			ray.intersect_TRI_count++;
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


}