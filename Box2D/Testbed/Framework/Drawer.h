/*
 * Original work Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_DRAWER_H
#define B2_DRAWER_H

#include <Box2D/Common/Math.h>

namespace box2d {

/// Color for debug drawing. Each value has the range [0,1].
struct Color
{
	Color() = default;
	constexpr Color(float_t ri, float_t gi, float_t bi, float_t ai = float_t{1}) : r(ri), g(gi), b(bi), a(ai) {}
	constexpr void Set(float_t ri, float_t gi, float_t bi, float_t ai = float_t{1}) { r = ri; g = gi; b = bi; a = ai; }
	float_t r, g, b, a;
};

class AABB;

class Drawer
{
public:
	using size_type = size_t;

	Drawer() = default;

	virtual ~Drawer() = default;

	/// Draw a closed polygon provided in CCW order.
	virtual void DrawPolygon(const Vec2* vertices, size_type vertexCount, const Color& color) = 0;

	/// Draw a solid closed polygon provided in CCW order.
	virtual void DrawSolidPolygon(const Vec2* vertices, size_type vertexCount, const Color& color) = 0;

	/// Draw a circle.
	virtual void DrawCircle(const Vec2& center, float_t radius, const Color& color) = 0;
	
	/// Draw a solid circle.
	virtual void DrawSolidCircle(const Vec2& center, float_t radius, const Vec2& axis, const Color& color) = 0;
	
	/// Draw a line segment.
	virtual void DrawSegment(const Vec2& p1, const Vec2& p2, const Color& color) = 0;

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	virtual void DrawTransform(const Transformation& xf) = 0;

	virtual void DrawPoint(const Vec2& p, float_t size, const Color& color) = 0;
	
	virtual void DrawString(int x, int y, const char* string, ...) = 0; 
	
	virtual void DrawString(const Vec2& p, const char* string, ...) = 0;
	
	virtual void DrawAABB(AABB* aabb, const Color& color) = 0;
	
	virtual void Flush() = 0;
	
	virtual void SetTranslation(Vec2 value) = 0;
	virtual Vec2 GetTranslation() const = 0;
};

} // namespace box2d

#endif
