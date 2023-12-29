#include "precomp.h"
#include "meshloader.h"


void Tmpl8::RenderUnityMesh(Tri tri[]) {
	FILE* file = fopen("../assets/unity.tri", "r");
	for (int t = 0; t < N; t++) {
		fscanf(file, "%f %f %f %f %f %f %f %f %f\n",
			&tri[t].vertex0.x, &tri[t].vertex0.y, &tri[t].vertex0.z,
			&tri[t].vertex1.x, &tri[t].vertex1.y, &tri[t].vertex1.z,
			&tri[t].vertex2.x, &tri[t].vertex2.y, &tri[t].vertex2.z);
		tri[t].centroid = tri[t].vertex0 + tri[t].vertex1 + tri[t].vertex2 * 0.3333f;
	}
}

void Tmpl8::RenderTriangles(Tri tri[]) {
	for (int i = 0; i < N; i++)
	{
		float3 r0 = float3(RandomFloat(), RandomFloat(), RandomFloat());
		float3 r1 = float3(RandomFloat(), RandomFloat(), RandomFloat());
		float3 r2 = float3(RandomFloat(), RandomFloat(), RandomFloat());
		tri[i].vertex0 = r0 * 9 - float3(5);
		tri[i].vertex1 = tri[i].vertex0 + r1, tri[i].vertex2 = tri[i].vertex0 + r2;
		tri[i].centroid = tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2 * 0.3333f;
		tri[i].ComputeAABB();
	}
}

void Tmpl8::Render_BrokenScreen(Tri tri[]) {
	float3 r0 = float3(RandomFloat(), RandomFloat(), RandomFloat());
	float3 r1 = float3(RandomFloat(), RandomFloat(), RandomFloat());
	float3 r2 = float3(RandomFloat(), RandomFloat(), RandomFloat());

	tri[0].vertex0 = r0;
	tri[0].vertex1 = r1;
	tri[0].vertex2 = r2;

	for (int i = 1; i < N; i++)
	{
		float3 new_r = float3(RandomFloat(), RandomFloat(), RandomFloat());
		tri[i].vertex0 = tri[i - 1].vertex2;
		tri[i].vertex1 = tri[i - 1].vertex0;
		tri[i].vertex2 = tri[i].vertex0 + new_r;
		tri[i].centroid = tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2 * 0.3333f;
		tri[i].ComputeAABB();
	}
}


