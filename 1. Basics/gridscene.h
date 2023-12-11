namespace Tmpl8 {

	class GridScene {
	public: 
		struct TriEx { float2 uv0, uv1, uv2; float3 N0, N1, N2; };

		int x_partition = 10;
		int y_partition = 10;
		int z_partition = 10;
		int GRID_SIZE = x_partition * y_partition * z_partition;

		struct A_GRID
		{
			float3 aabbMin, aabbMax;
			uint triCount = 0;
			vector<Tri> triangles;
		};

		GridScene() = default;

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

		void RenderTriangles() {
			for (int i = 0; i < N; i++)
			{
				float3 r0 = float3(RandomFloat(), RandomFloat(), RandomFloat());
				float3 r1 = float3(RandomFloat(), RandomFloat(), RandomFloat());
				float3 r2 = float3(RandomFloat(), RandomFloat(), RandomFloat());
				tri[i].vertex0 = r0 * 9 - float3(5);
				tri[i].vertex1 = tri[i].vertex0 + r1, tri[i].vertex2 = tri[i].vertex0 + r2;
			}
		}

		void RenderUnityMesh() {
			FILE* file = fopen("../assets/unity.tri", "r");
			for (int t = 0; t < N; t++) {
				fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
					&tri[t].vertex0.x, &tri[t].vertex0.y, &tri[t].vertex0.z,
					&tri[t].vertex1.x, &tri[t].vertex1.y, &tri[t].vertex1.z,
					&tri[t].vertex2.x, &tri[t].vertex2.y, &tri[t].vertex2.z);
			}
		}

		void Read_Mesh_OBJ() {
			std::string inputfile = "../assets/m1354.obj";

			tinyobj::ObjReader reader;
			tinyobj::ObjReaderConfig reader_config;

			if (!reader.ParseFromFile(inputfile, reader_config)) {
				if (!reader.Error().empty()) {
					std::cerr << "TinyObjReader: " << reader.Error();
				}
				exit(1);
			}

			if (!reader.Warning().empty()) {
				std::cout << "TinyObjReader: " << reader.Warning();
			}

			auto& attrib = reader.GetAttrib();
			auto& shapes = reader.GetShapes();
			auto& materials = reader.GetMaterials();

			// Loop over shapes
			for (size_t s = 0; s < shapes.size(); s++) {
				// Loop over faces(polygon)
				size_t index_offset = 0;
				for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
					size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

					// Loop over vertices in the face.
					for (size_t v = 0; v < fv; v++) {
						// access to vertex
						tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
						tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
						tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
						tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

						if (v == 0) {
							tri[f].vertex0 = float3(vx, vy, vz) * 15;
						}
						if (v == 1) {
							tri[f].vertex1 = float3(vx, vy, vz) * 15;
						}
						if (v == 2) {
							tri[f].vertex2 = float3(vx, vy, vz) * 15;
						}

					}
					index_offset += fv;

					// per-face material
					shapes[s].mesh.material_ids[f];
				}
				break;
			}
		}


		void IntersectGRID(Ray& ray)
		{
			ray.traversal_count++;
			if (!IntersectAABB(ray, glb_aabbMin, glb_aabbMax))
				return;


			for (int grid_idx = 0; grid_idx < GRID_SIZE; grid_idx++) {
				if (grids[grid_idx].triCount < 1)
					continue;
				if (!IntersectAABB(ray, grids[grid_idx].aabbMin, grids[grid_idx].aabbMax))
					continue;

				for (Tri& currentTri : grids[grid_idx].triangles) {
					currentTri.IntersectTri(ray);
				}

			}
		}

		bool isIntersectingAABB(const float3 p1, const float3 p2, const float3& aabbMin, const float3& aabbMax) {
			// Check if the line segment is outside the AABB along any axis
			if (p2.x < aabbMin.x && p1.x < aabbMin.x) return false;
			if (p2.x > aabbMax.x && p1.x > aabbMax.x) return false;

			if (p2.y < aabbMin.y && p1.y < aabbMin.y) return false;
			if (p2.y > aabbMax.y && p1.y > aabbMax.y) return false;

			if (p2.z < aabbMin.z && p1.z < aabbMin.z) return false;
			if (p2.z > aabbMax.z && p1.z > aabbMax.z) return false;

			// Check if the line intersects the AABB
			float t1 = (aabbMin.x - p1.x) / (p2.x - p1.x);
			float t2 = (aabbMax.x - p1.x) / (p2.x - p1.x);
			float t3 = (aabbMin.y - p1.y) / (p2.y - p1.y);
			float t4 = (aabbMax.y - p1.y) / (p2.y - p1.y);
			float t5 = (aabbMin.z - p1.z) / (p2.z - p1.z);
			float t6 = (aabbMax.z - p1.z) / (p2.z - p1.z);

			float tMin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
			float tMax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

			// If the intersection interval is valid, there is an intersection
			return tMax >= 0 && tMin <= 1;
		}

		bool vertice_in_grid(A_GRID a_grid, Tri a_tri) {
			return isIntersectingAABB(a_tri.vertex0, a_tri.vertex1, a_grid.aabbMin, a_grid.aabbMax) ||
				isIntersectingAABB(a_tri.vertex0, a_tri.vertex2, a_grid.aabbMin, a_grid.aabbMax) ||
				isIntersectingAABB(a_tri.vertex1, a_tri.vertex2, a_grid.aabbMin, a_grid.aabbMax);

		}
		void BuildGRID()
		{

			for (int i = 0; i < N; i++)
			{
				if (tri[i].vertex0.x < -100 ||
					tri[i].vertex0.y < -100 ||
					tri[i].vertex0.z < -100
					)
				{
					cout << "wtf?";
					break;
				}
				tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * 0.3333f;
				glb_aabbMax = fmaxf(glb_aabbMax, tri[i].vertex0);
				glb_aabbMax = fmaxf(glb_aabbMax, tri[i].vertex1);
				glb_aabbMax = fmaxf(glb_aabbMax, tri[i].vertex2);


				glb_aabbMin = fminf(glb_aabbMin, tri[i].vertex0);
				glb_aabbMin = fminf(glb_aabbMin, tri[i].vertex1);
				glb_aabbMin = fminf(glb_aabbMin, tri[i].vertex2);


			}

			// * 1.2 so there will be some border around the model
			float unit_length_x = (glb_aabbMax.x - glb_aabbMin.x) * 1.2 / x_partition;
			float unit_length_y = (glb_aabbMax.y - glb_aabbMin.y) * 1.2 / y_partition;
			float unit_length_z = (glb_aabbMax.z - glb_aabbMin.z) * 1.2 / z_partition;
			// construct grid
			// add triangles to grid
			//iterate grid and triangles.
			cout << unit_length_x << endl;
			cout << unit_length_y << endl;
			cout << unit_length_z << endl;
			int grid_idx = 0;
			for (int index_z = 0; index_z < z_partition; index_z++) {
				for (int index_y = 0; index_y < y_partition; index_y++) {
					for (int index_x = 0; index_x < z_partition; index_x++) {
						//cout << index_x << "," << index_y << " "<<end;
						grids[grid_idx].triCount = 0;
						cout << glb_aabbMin.x + index_x * unit_length_x << "," << glb_aabbMin.y + index_y * unit_length_y << "," << glb_aabbMin.z + index_z * unit_length_z << "   ";
						grids[grid_idx].aabbMin = float3(
							glb_aabbMin.x + index_x * unit_length_x,
							glb_aabbMin.y + index_y * unit_length_y,
							glb_aabbMin.z + index_z * unit_length_z);
						grids[grid_idx].aabbMax = float3(
							glb_aabbMin.x + (index_x + 1) * unit_length_x,
							glb_aabbMin.y + (index_y + 1) * unit_length_y,
							glb_aabbMin.z + (index_z + 1) * unit_length_z);
						grid_idx++;
					}
					cout << endl;
				}
				cout << endl;
			}

			for (int grid_index = 0; grid_index < GRID_SIZE; grid_index++) {
				for (int i = 0; i < N; i++) {
					if (vertice_in_grid(grids[grid_index], tri[i])) {
						grids[grid_index].triangles.push_back(tri[i]);
						grids[grid_index].triCount++;
					}
				}
				if (grids[grid_index].triCount > 0) {
					cout << "IDX:" << grid_index << " " << grids[grid_index].triCount << " " << endl;
				}
			}

		}
		Tri tri[N];
		A_GRID* grids = new A_GRID[GRID_SIZE];
		float3 glb_aabbMin = float3(1e30f);
		float3 glb_aabbMax = float3(-1e30f);
	};
}
