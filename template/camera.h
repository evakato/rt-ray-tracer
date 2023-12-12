#pragma once

// default screen resolution
#define SCRWIDTH	1024
#define SCRHEIGHT	640
// #define FULLSCREEN
// #define DOUBLESIZE

namespace Tmpl8 {

class Camera
{
public:
	Camera()
	{
		// setup a basic view frustum
		camPos = float3( 0, 0, -2 );
		camTarget = float3( 0, 0, -1 );
		topLeft = float3( -aspect, 1, 0 );
		topRight = float3( aspect, 1, 0 );
		bottomLeft = float3( -aspect, -1, 0 );
	}
	Ray GetPrimaryRay( const float x, const float y )
	{
		// calculate pixel position on virtual screen plane
		const float u = (float)x * (1.0f / SCRWIDTH);
		const float v = (float)y * (1.0f / SCRHEIGHT);
		const float3 P = topLeft + u * (topRight - topLeft) + v * (bottomLeft - topLeft);
		return Ray( camPos, normalize( P - camPos ) );
	}
	bool HandleInput( const float t )
	{
		if (!WindowHasFocus()) return false;
		float speed = 0.0025f * t;
		float3 ahead = normalize( camTarget - camPos );
		float3 tmpUp( 0, 1, 0 );
		float3 right = normalize( cross( tmpUp, ahead ) );
		float3 up = normalize( cross( ahead, right ) );
		bool changed = false;
		if (IsKeyDown( GLFW_KEY_A )) camPos -= speed * 2 * right, changed = true;
		if (IsKeyDown( GLFW_KEY_D )) camPos += speed * 2 * right, changed = true;
		if (IsKeyDown( GLFW_KEY_W )) camPos += speed * 2 * ahead, changed = true;
		if (IsKeyDown( GLFW_KEY_S )) camPos -= speed * 2 * ahead, changed = true;
		if (IsKeyDown( GLFW_KEY_R )) camPos += speed * 2 * up, changed = true;
		if (IsKeyDown( GLFW_KEY_F )) camPos -= speed * 2 * up, changed = true;
		camTarget = camPos + ahead;
		if (IsKeyDown( GLFW_KEY_UP )) camTarget += speed * up, changed = true;
		if (IsKeyDown( GLFW_KEY_DOWN )) camTarget -= speed * up, changed = true;
		if (IsKeyDown( GLFW_KEY_LEFT )) camTarget -= speed * right, changed = true;
		if (IsKeyDown(GLFW_KEY_RIGHT)) camTarget += speed * right, changed = true;
		if (IsKeyDown(GLFW_KEY_1)) camTarget = float3(-2.66256, 2.25068, -1.41421), camPos = float3(-3.09958, 3.07972, -1.76307), changed = true;
		if (IsKeyDown(GLFW_KEY_2)) camTarget = float3(-2.70643, -0.0108962, 2.23854), camPos=float3(-3.20559, 0.0220974, 3.10442), changed = true;
		if (IsKeyDown(GLFW_KEY_3)) camTarget = float3(0.946654, -0.0176587, 1.18748), camPos=float3(1.82779, 0.0119818, 1.65942), changed = true;

		if (!changed) return false;
		ahead = normalize( camTarget - camPos );
		up = normalize( cross( ahead, right ) );
		right = normalize( cross( up, ahead ) );
		topLeft = camPos + 2 * ahead - aspect * right + up;
		topRight = camPos + 2 * ahead + aspect * right + up;
		bottomLeft = camPos + 2 * ahead - aspect * right - up;
		return true;
	}
	float aspect = (float)SCRWIDTH / (float)SCRHEIGHT;
	float3 camPos, camTarget;
	float3 topLeft, topRight, bottomLeft;
};

}