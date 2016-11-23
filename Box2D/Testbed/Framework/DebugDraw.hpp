/*
 * Original work Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef DEBUGDRAW_H
#define DEBUGDRAW_H

#include <Box2D/Box2D.hpp>
#include "Drawer.hpp"

namespace box2d {

struct GLRenderPoints;
struct GLRenderLines;
struct GLRenderTriangles;

struct AABB;

struct ProjectionMatrix
{
	float m[16];
};
	
//
struct Camera
{
	Vec2 m_center = Vec2(0.0f, 20.0f);
	float_t m_extent = 25.0f;
	float_t m_zoom = 1.0f;
	int32 m_width = 1280;
	int32 m_height = 800;
};

Vec2 ConvertScreenToWorld(const Camera& camera, const Vec2 screenPoint);
Vec2 ConvertWorldToScreen(const Camera& camera, const Vec2 worldPoint);
ProjectionMatrix GetProjectionMatrix(const Camera& camera, float_t zBias);

class DebugDraw : public Drawer
{
public:
	DebugDraw(Camera& camera);

	virtual ~DebugDraw() noexcept;
	
	void DrawPolygon(const Vec2* vertices, size_type vertexCount, const Color& color) override;

	void DrawSolidPolygon(const Vec2* vertices, size_type vertexCount, const Color& color) override;

	void DrawCircle(const Vec2& center, float_t radius, const Color& color) override;

	void DrawSolidCircle(const Vec2& center, float_t radius, const Color& color) override;

	void DrawSegment(const Vec2& p1, const Vec2& p2, const Color& color) override;

    void DrawPoint(const Vec2& p, float_t size, const Color& color) override;

    void DrawString(int x, int y, const char* string, ...) override; 

    void DrawString(const Vec2& p, const char* string, ...) override;

    void Flush() override;
	
	Vec2 GetTranslation() const override;

	void SetTranslation(Vec2 value) override;

private:
	void DrawTriangle(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Color& color);

	Camera& m_camera;
	GLRenderPoints* m_points;
    GLRenderLines* m_lines;
    GLRenderTriangles* m_triangles;
	int m_circleParts = 16;
	float_t m_cosInc;
	float_t m_sinInc;
};

} // namespace box2d

#endif
