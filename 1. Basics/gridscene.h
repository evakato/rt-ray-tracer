#pragma once
namespace Tmpl8 {

	class GridScene {
	public:
		struct TriEx { float2 uv0, uv1, uv2; float3 N0, N1, N2; };

		int x_partition = 3;
		int y_partition = 3;
		int z_partition = 3;
		int GRID_SIZE = x_partition * y_partition * z_partition;

		struct A_GRID
		{
			float3 aabbMin, aabbMax;
			uint triCount = 0;
			vector<Tri> triangles;
		};


		GridScene() = default;

		Tri tri[N];
		A_GRID* grids = new A_GRID[GRID_SIZE];
		float3 glb_aabbMin = float3(1e30f);
		float3 glb_aabbMax = float3(-1e30f);

		inline float IntersectAABB(const Ray& ray, const float3 bmin, const float3 bmax);

		void RenderUnityMesh();

		void Read_Mesh_OBJ();

		void RenderTriangles();
		void IntersectGRID(Ray& ray);


		bool isIntersectingAABB(const float3 p1, const float3 p2, const float3& aabbMin, const float3& aabbMax);

		bool vertice_in_grid(A_GRID a_grid, Tri a_tri);
		void BuildGRID();


	};
}
