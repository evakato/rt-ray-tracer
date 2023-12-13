#include <precomp.h>

#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc

//#include "scene.h"
#include "tiny_obj_loader.h"

void GridScene::RenderUnityMesh() {
	FILE* file = fopen("../assets/unity.tri", "r");
	for (int t = 0; t < N; t++) {
		fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
			&tri[t].vertex0.x, &tri[t].vertex0.y, &tri[t].vertex0.z,
			&tri[t].vertex1.x, &tri[t].vertex1.y, &tri[t].vertex1.z,
			&tri[t].vertex2.x, &tri[t].vertex2.y, &tri[t].vertex2.z);
	}
}

bool GridScene::WithinAABB(float3 v) {
	return v[0] >= glb_aabbMin[0] && v[0] <= glb_aabbMax[0]
		&& v[1] >= glb_aabbMin[1] && v[1] <= glb_aabbMax[1]
		&& v[2] >= glb_aabbMin[2] && v[2] <= glb_aabbMax[2];
}

// Reference: https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-acceleration-structure/grid.html
void GridScene::IntersectGRID(Ray& ray)
{
	//ray.traversal_count++;
	float t = IntersectAABB(ray, glb_aabbMin, glb_aabbMax);
	if (t == 1e30f) return; // miss

	float3 deltaT, tMax;
	int3 cell, step, exit;
	for (int i = 0; i < 3; i++) {
		float rayOrigCell = ((ray.O[i] + ray.D[i] * t) - glb_aabbMin[i]);
		cell[i] = clamp<uint32_t>(floor(rayOrigCell / cellSize[i]), 0, x_partition - 1);

		if (ray.D[i] < 0) {
			deltaT[i] = -cellSize[i] / ray.D[i];
			tMax[i] = t + (cell[i] * cellSize[i] - rayOrigCell) * (1 / ray.D[i]);
			exit[i] = -1;
			step[i] = -1;
		}
		else {
			deltaT[i] = cellSize[i] / ray.D[i];
			tMax[i] = t + ((cell[i] + 1) * cellSize[i] - rayOrigCell) * (1 / ray.D[i]);
			exit[i] = x_partition;
			step[i] = 1;
		}
	}

	while (1) {
		int grid_idx = cell[0] + cell[1] * x_partition + cell[2] * x_partition * y_partition;
		for (Tri& currentTri : grids[grid_idx].triangles) {
			if (currentTri.IntersectTriGrid(ray)) return;
		}
		uint8_t k =
			((tMax[0] < tMax[1]) << 2) +
			((tMax[0] < tMax[2]) << 1) +
			((tMax[1] < tMax[2]));
		static const uint8_t map[8] = { 2, 1, 2, 1, 2, 2, 0, 0 };
		uint8_t axis = map[k];

		if (ray.t < tMax[axis]) break;
		cell[axis] += step[axis];
		if (cell[axis] == exit[axis]) break;
		tMax[axis] += deltaT[axis];
	}

}

inline float GridScene::IntersectAABB(const Ray& ray, const float3 bmin, const float3 bmax)
{
	float tx1 = (bmin.x - ray.O.x) * ray.rD.x, tx2 = (bmax.x - ray.O.x) * ray.rD.x;
	float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
	float ty1 = (bmin.y - ray.O.y) * ray.rD.y, ty2 = (bmax.y - ray.O.y) * ray.rD.y;
	tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
	float tz1 = (bmin.z - ray.O.z) * ray.rD.z, tz2 = (bmax.z - ray.O.z) * ray.rD.z;
	tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
	if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin; else return 1e30f;
}

void GridScene::Read_Mesh_OBJ() {
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

void GridScene::RenderTriangles() {
	for (int i = 0; i < N; i++)
	{
		float3 r0 = float3(RandomFloat(), RandomFloat(), RandomFloat());
		float3 r1 = float3(RandomFloat(), RandomFloat(), RandomFloat());
		float3 r2 = float3(RandomFloat(), RandomFloat(), RandomFloat());
		GridScene::tri[i].vertex0 = r0 * 9 - float3(5);
		GridScene::tri[i].vertex1 = GridScene::tri[i].vertex0 + r1, GridScene::tri[i].vertex2 = GridScene::tri[i].vertex0 + r2;
	}
}

bool  GridScene::vertice_in_grid(A_GRID a_grid, Tri a_tri)
{
	return isIntersectingAABB(a_tri.vertex0, a_tri.vertex1, a_grid.aabbMin, a_grid.aabbMax) ||
		isIntersectingAABB(a_tri.vertex0, a_tri.vertex2, a_grid.aabbMin, a_grid.aabbMax) ||
		isIntersectingAABB(a_tri.vertex1, a_tri.vertex2, a_grid.aabbMin, a_grid.aabbMax);

}

bool GridScene::isIntersectingAABB(const float3 p1, const float3 p2, const float3& aabbMin, const float3& aabbMax) {
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


void GridScene::BuildGRID()
{
	for (int i = 0; i < N; i++)
	{
		glb_aabbMax = fmaxf(glb_aabbMax, tri[i].vertex0);
		glb_aabbMax = fmaxf(glb_aabbMax, tri[i].vertex1);
		glb_aabbMax = fmaxf(glb_aabbMax, tri[i].vertex2);
		glb_aabbMin = fminf(glb_aabbMin, tri[i].vertex0);
		glb_aabbMin = fminf(glb_aabbMin, tri[i].vertex1);
		glb_aabbMin = fminf(glb_aabbMin, tri[i].vertex2);
	}

	cellSize[0] = (glb_aabbMax.x - glb_aabbMin.x) / x_partition;
	cellSize[1] = (glb_aabbMax.y - glb_aabbMin.y) / y_partition;
	cellSize[2] = (glb_aabbMax.z - glb_aabbMin.z) / z_partition;
	// construct grid
	// add triangles to grid
	//iterate grid and triangles.
	cout << cellSize[0] << endl;
	cout << cellSize[1] << endl;
	cout << cellSize[2] << endl;
	int grid_idx = 0;
	for (int index_z = 0; index_z < z_partition; index_z++) {
		for (int index_y = 0; index_y < y_partition; index_y++) {
			for (int index_x = 0; index_x < z_partition; index_x++) {
				//cout << index_x << "," << index_y << " "<<end;
				grids[grid_idx].aabbMin = float3(
					glb_aabbMin.x + index_x * cellSize[0],
					glb_aabbMin.y + index_y * cellSize[1],
					glb_aabbMin.z + index_z * cellSize[2]);
				grids[grid_idx].aabbMax = float3(
					glb_aabbMin.x + (index_x + 1) * cellSize[0],
					glb_aabbMin.y + (index_y + 1) * cellSize[1],
					glb_aabbMin.z + (index_z + 1) * cellSize[2]);
				grid_idx++;
			}
		}
	}

	for (int grid_index = 0; grid_index < GRID_SIZE; grid_index++) {
		for (int i = 0; i < N; i++) {
			if (vertice_in_grid(grids[grid_index], tri[i])) {
				grids[grid_index].triangles.push_back(tri[i]);
			}
		}
		if (grids[grid_index].triangles.size() > 0) {
			cout << "IDX:" << grid_index << " " << grids[grid_index].triangles.size() << " " << endl;
		}
	}

}