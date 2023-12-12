#include <precomp.h>
#include "tiny_obj_loader.h"

void BVHScene::Read_Mesh_OBJ() {
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

