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
	}
	void KeyDown( int key ) { /* implement if you want to handle keys */ }
	// data members
	void AnimateScene();
	int2 mousePos;
	BVHInstance bvhInstance[256];
	TLAS tlas;
	float3* position, * direction, * orientation;
	float4* accumulator;
	Camera camera;
	string RT_MODE = "BVH";
	string SCENE_CODE = "1"; // 1 for the random triangle and 2 for the unity.tri
	bool animating = true;
	float anim_time = 0;
};

} // namespace Tmpl8