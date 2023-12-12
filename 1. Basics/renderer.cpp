#include "precomp.h"


// Initialize the renderer
void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
	memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );
	//bvhScene.RenderUnityMesh();
	//bvhScene.BuildBVH();
	kdtreeScene.RenderUnityMesh();
	kdtreeScene.BuildKDTree();
	//gridScene.Read_Mesh_OBJ();
	//gridScene.BuildGRID();
}

// Evaluate light transport
float3 Renderer::Trace( Ray& ray )
{
	  //bvhScene.IntersectBVH(ray);
	  kdtreeScene.IntersectKD(ray);

	//kdtreeScene.FindNearestTri(ray);
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
	Timer t;
	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)

	int max_traversal = 0, min_traversal = 0;
	float avg_traversal = 0;
	int max_intersect_TRI = 0, min_intersect_TRI = 0;
	float avg_intersect_TRI = 0;
	int max_intersect_AABB = 0, min_intersect_AABB = 0;
	float avg_intersect_AABB = 0;
	float total_traversal = 0;
	float total_intersect_TRI = 0;
	float total_intersect_AABB = 0;


#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line

		for (int x = 0; x < SCRWIDTH; x++)
		{
			auto primary_ray = camera.GetPrimaryRay((float)x, (float)y);
			auto pixel_color = Trace(primary_ray);
			float4 pixel = float4(pixel_color, 0);
			// translate accumulator contents to rgb32 pixels
			screen->pixels[x + y * SCRWIDTH] = RGBF32_to_RGB8(&pixel);

			accumulator[x + y * SCRWIDTH] = pixel;

			max_traversal = max(max_traversal, primary_ray.traversal_count);
			min_traversal = min(min_traversal, primary_ray.traversal_count);
			total_traversal += primary_ray.traversal_count;

			max_intersect_TRI = max(max_intersect_TRI, primary_ray.intersect_TRI_count);
			min_intersect_TRI = min(min_intersect_TRI, primary_ray.intersect_TRI_count);
			total_intersect_TRI += primary_ray.intersect_TRI_count;

			max_intersect_AABB = max(max_intersect_AABB, primary_ray.intersect_AABB_count);
			min_intersect_AABB = min(min_intersect_AABB, primary_ray.intersect_AABB_count);
			total_intersect_AABB += primary_ray.intersect_AABB_count;
		}
	}
	avg_traversal = total_traversal / SCRHEIGHT / SCRWIDTH;
	avg_intersect_AABB = total_intersect_AABB / SCRHEIGHT / SCRWIDTH;
	avg_intersect_TRI = total_intersect_TRI / SCRHEIGHT / SCRWIDTH;

	cout << "TRAVERSAL" << endl;
	cout << "AVG:" << avg_traversal << " ";
	cout << "MIN:" << min_traversal << " ";
	cout << "MAX:" << max_traversal << " ";
	cout << endl << endl;

	cout << "INTERSECT_TRI" << endl;
	cout << "AVG:" << avg_intersect_TRI << " ";
	cout << "MIN:" << min_intersect_TRI << " ";
	cout << "MAX:" << max_intersect_TRI << " ";
	cout << endl << endl;

	cout << "INTERSECT_AABB" << endl;
	cout << "AVG:" << avg_intersect_AABB << " ";
	cout << "MIN:" << min_intersect_AABB << " ";
	cout << "MAX:" << max_intersect_AABB << " ";
	cout << endl << endl;

	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000.0f / avg, rps = (SCRWIDTH * SCRHEIGHT) / avg;
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