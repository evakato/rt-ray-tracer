namespace Tmpl8 {
	class Mesh
	{
	public:
		Tri* tri = 0;			// triangle data for intersection
		TriEx* triEx = 0;		// triangle data for shading
		int triCount = 0;
		BVHScene* bvh = 0;
		Surface* texture = 0;
		float3* P = 0, * N = 0;

		Mesh() = default;
		Mesh(uint primCount) {
			// basic constructor, for top-down TLAS construction
			tri = (Tri*)_aligned_malloc(primCount * sizeof(Tri), 64);
			memset(tri, 0, primCount * sizeof(Tri));
			triEx = (TriEx*)_aligned_malloc(primCount * sizeof(TriEx), 64);
			memset(triEx, 0, primCount * sizeof(TriEx));
			triCount = primCount;
		}
		Mesh(const char* objFile, const char* texFile)
		{
			// bare-bones obj file loader; only supports very basic meshes
			tri = new Tri[25000];
			triEx = new TriEx[25000];
			float2* UV = new float2[11042]; // enough for dragon.obj
			N = new float3[11042], P = new float3[11042];
			int UVs = 0, Ns = 0, Ps = 0, a, b, c, d, e, f, g, h, i;
			FILE* file = fopen(objFile, "r");
			if (!file) return; // file doesn't exist
			while (!feof(file))
			{
				char line[512] = { 0 };
				fgets(line, 511, file);
				if (line == strstr(line, "vt "))
					sscanf(line + 3, "%f %f", &UV[UVs].x, &UV[UVs].y), UVs++;
				else if (line == strstr(line, "vn "))
					sscanf(line + 3, "%f %f %f", &N[Ns].x, &N[Ns].y, &N[Ns].z), Ns++;
				else if (line[0] == 'v')
					sscanf(line + 2, "%f %f %f", &P[Ps].x, &P[Ps].y, &P[Ps].z), Ps++;
				if (line[0] != 'f') continue; else
					sscanf(line + 2, "%i/%i/%i %i/%i/%i %i/%i/%i",
						&a, &b, &c, &d, &e, &f, &g, &h, &i);
				tri[triCount].vertex0 = P[a - 1], triEx[triCount].N0 = N[c - 1];
				tri[triCount].vertex1 = P[d - 1], triEx[triCount].N1 = N[f - 1];
				tri[triCount].vertex2 = P[g - 1], triEx[triCount].N2 = N[i - 1];
				triEx[triCount].uv0 = UV[b - 1], triEx[triCount].uv1 = UV[e - 1];
				triEx[triCount++].uv2 = UV[h - 1];
			}
			fclose(file);
			bvh = new BVHScene(this);
			texture = new Surface(texFile);
		}

	};

}