#pragma once

namespace Tmpl8
{

class Renderer : public TheApp
{
public:
	// game flow methods
	void Init();
	float3 Trace( Ray& ray );
	void Tick( float deltaTime );
	void UI();
	void Shutdown() { /* implement if you want to do things on shutdown */ }
	// input handling
	void MouseUp( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseDown( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseMove( int x, int y ) { mousePos.x = x, mousePos.y = y; }
	void MouseWheel( float y ) { /* implement if you want to handle the mouse wheel */ }
	void KeyUp( int key ) { /* implement if you want to handle keys */ 
		if (IsKeyDown(GLFW_KEY_B)) RT_MODE = "BVH";
		if (IsKeyDown(GLFW_KEY_K)) RT_MODE = "KDTREE";
		if (IsKeyDown(GLFW_KEY_G)) RT_MODE = "GRID";

		if (IsKeyDown(GLFW_KEY_F1)) {
			kdtreeScene.RenderUnityMesh();
			kdtreeScene.BuildKDTree();
		//	bvhScene.RenderUnityMesh();
		//	bvhScene.BuildBVH();
		}
		if (IsKeyDown(GLFW_KEY_F2)) {
			kdtreeScene.RenderTriangles();
			kdtreeScene.BuildKDTree();
		//	bvhScene.RenderTriangles();
		//	bvhScene.BuildBVH();
		}
		if (IsKeyDown(GLFW_KEY_F3)) {
			camera.camPos = float3(-8.32943, -23.6732, -32.2682);
			camera.camTarget = float3(-7.875, -23.1848, -31.5232);

		}
	}
	void KeyDown( int key ) { /* implement if you want to handle keys */ }
	// data members
	void RenderUnityMesh();
	int2 mousePos;
	float4* accumulator;
	//Scene scene;
	BVHScene bvhScene;
	KDTreeScene kdtreeScene;
	GridScene gridScene;
	Camera camera;
	string RT_MODE = "BVH";
	string SCENE_CODE = "3"; // 1 for the random triangle and 2 for the unity.tri and 3 for the Broken_screen
	bool animating = true;
	float anim_time = 0;
};

} // namespace Tmpl8