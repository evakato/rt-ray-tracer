#include "precomp.h"


// Initialize the renderer
void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
	memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );
	// kdtreeScene.RenderTriangles();
	// kdtreeScene.RenderUnityMesh();
	// bvhScene.RenderUnityMesh();
	bvhScene.Read_Mesh_OBJ();
	bvhScene.BuildBVH();
	// kdtreeScene.BuildKDTree();
	// gridScene.Read_Mesh_OBJ();
	//gridScene.BuildGRID();
}

// Evaluate light transport
float3 Renderer::Trace( Ray& ray )
{
	bvhScene.IntersectBVH(ray);
	//kdtreeScene.FindNearestTri(ray);
	// kdtreeScene.IntersectKD(ray);
	// gridScene.IntersectGRID(ray);
	if (ray.t < 1e30f) return 0.1f * float3(ray.t, ray.t, ray.t);
	return 0;
#if 0
	scene.FindNearest( ray );
	if (ray.objIdx == -1) return 0; // or a fancy sky color
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal( ray.objIdx, I, ray.D );
	float3 albedo = scene.GetAlbedo( ray.objIdx, I );
	/* visualize normal */ // return (N + 1) * 0.5f;
	/* visualize distance */ // return 0.1f * float3( ray.t, ray.t, ray.t );
	/* visualize albedo */ return albedo;
#endif
}

// Main application tick function - Executed once per frame
void Renderer::Tick( float deltaTime )
{
	// animation
	//if (animating) scene.SetTime( anim_time += deltaTime * 0.002f );
	// pixel loop
	float TOTAL_TRAVERSAL = 0, TOTAL_AABB_INTSCT = 0, TOTAL_TRI_INTSCT = 0;
	float AVG_TRAVERSAL = 0, AVG_AABB_INTSCT = 0, AVG_TRI_INTSCT = 0;
	int MIN_TRI_INTSCT = 0, MAX_TRI_INTSCT = 0;
	int MIN_AABB_INTSCT = 0, MAX_AABB_INTSCT = 0;
	int MIN_TRAVERSAL=0, MAX_TRAVERSAL = 0;
	
	Timer t;
	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{	
			auto ray = camera.GetPrimaryRay((float)x, (float)y) ;
			float4 pixel = float4( Trace(ray), 0);
			screen->pixels[x + y * SCRWIDTH] = RGBF32_to_RGB8( &pixel );
			accumulator[x + y * SCRWIDTH] = pixel;
			TOTAL_AABB_INTSCT += ray.intersect_AABB_count;
			TOTAL_TRI_INTSCT += ray.intersect_TRI_count;
			TOTAL_TRAVERSAL += ray.traversal_count;
			MIN_TRI_INTSCT = min(MIN_TRI_INTSCT, ray.intersect_TRI_count);
			MIN_AABB_INTSCT = min(MIN_AABB_INTSCT, ray.intersect_AABB_count);
			MIN_TRAVERSAL = min(MIN_TRAVERSAL, ray.traversal_count);

			MAX_TRI_INTSCT = max(MAX_TRI_INTSCT, ray.intersect_TRI_count);
			MAX_AABB_INTSCT = max(MAX_AABB_INTSCT, ray.intersect_AABB_count);
			MAX_TRAVERSAL = max(MAX_TRAVERSAL, ray.traversal_count);

		}

	}
	AVG_TRAVERSAL = TOTAL_TRAVERSAL / SCRWIDTH / SCRHEIGHT;
	AVG_TRI_INTSCT = TOTAL_TRI_INTSCT / SCRWIDTH / SCRHEIGHT;
	AVG_AABB_INTSCT = TOTAL_AABB_INTSCT / SCRWIDTH / SCRHEIGHT;
	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000.0f / avg, rps = (SCRWIDTH * SCRHEIGHT) / avg;
	cout << "TRAVERSAL:" << endl;
	cout << "MIN:" << MIN_TRAVERSAL << "   MAX:" << MAX_TRAVERSAL << "   AVG:" << AVG_TRAVERSAL<<endl<<endl;
	cout << "AABB_INTERSECTION:" << endl;
	cout << "MIN:" << MIN_AABB_INTSCT << "   MAX:" << MAX_AABB_INTSCT << "   AVG:" << AVG_AABB_INTSCT<<endl<<endl;
	cout << "TRI_INTERSECTION:" << endl;
	cout << "TRI:" << endl;
	cout << "MIN:" << MIN_TRI_INTSCT << "   MAX:" << MAX_TRI_INTSCT << "   AVG:" << AVG_TRI_INTSCT<<endl<<endl;

	printf( "%5.2fms (%.1ffps) - %.1fMrays/s\n", avg, fps, rps / 1000 );
	// handle user input
	camera.HandleInput( deltaTime );
}

// -----------------------------------------------------------
// Update user interface (imgui)
// -----------------------------------------------------------
void Renderer::UI()
{
	// animation toggle
	ImGui::Checkbox( "Animate scene", &animating );
	// ray query on mouse
	//Ray r = camera.GetPrimaryRay( (float)mousePos.x, (float)mousePos.y );
	//scene.FindNearest( r );
	//ImGui::Text( "Object id: %i", r.objIdx );
}