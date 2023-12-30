#include "precomp.h"


// Initialize the renderer
void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
	memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );

	BVHScene* bvh = new BVHScene("../assets/armadillo.tri", 30000);
	for (int i = 0; i < 256; i++)
		bvhInstance[i] = BVHInstance(bvh);
	tlas = TLAS(bvhInstance, 256);
	// set up spacy armadillo army
	position = new float3[256];
	direction = new float3[256];
	orientation = new float3[256];
	for (int i = 0; i < 256; i++)
	{
		position[i] = float3(RandomFloat(), RandomFloat(), RandomFloat()) - 0.5f;
		position[i] *= 4;
		direction[i] = normalize(position[i]) * 0.05f;
		orientation[i] = float3(RandomFloat(), RandomFloat(), RandomFloat()) * 2.5f;
	}

}

// Evaluate light transport
float3 Renderer::Trace( Ray& ray )
{
	tlas.Intersect(ray);
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
	for (int i = 0; i < 256; i++)
	{
		mat4 R = mat4::RotateX(orientation[i].x) *
			mat4::RotateY(orientation[i].y) *
			mat4::RotateZ(orientation[i].z) * mat4::Scale(0.2f);
		bvhInstance[i].SetTransform(mat4::Translate(position[i]) * R);
		position[i] += direction[i], orientation[i] += direction[i];
		if (position[i].x < -3 || position[i].x > 3) direction[i].x *= -1;
		if (position[i].y < -3 || position[i].y > 3) direction[i].y *= -1;
		if (position[i].z < -3 || position[i].z > 3) direction[i].z *= -1;
	}
	Timer t;
	tlas.Build();

	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
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
		}
	}

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