/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_CIRCLE_SHAPE_H
#define B2_CIRCLE_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.h>

namespace box2d {

/// Circle shape.
class CircleShape : public Shape
{
public:
	
	static float_t GetDefaultRadius() noexcept
	{
		return 0;
	}

	/// Initializing constructor.
	///
	/// @note Behavior is undefined if a negative radius is given.
	///
	/// @param radius Radius of the circle shape (in meters).
	/// @param location Location of the center of this shape.
	constexpr explicit CircleShape(float_t radius = GetDefaultRadius(), Vec2 location = Vec2_zero) noexcept:
		Shape{e_circle, radius}, m_location{location}
	{
		// Intentionally empty.
	}

	CircleShape(const CircleShape&) = default;

	CircleShape& operator=(const CircleShape& other) = default;

	/// Gets the "radius" of the shape.
	/// @return Non-negative distance.
	float_t GetRadius() const noexcept { return GetVertexRadius(); }
	
	void SetRadius(float_t radius) noexcept
	{
		SetVertexRadius(radius);
	}

	/// Gets the location of the center of this circle shape.
	/// @return The origin (0, 0) unless explicitly set otherwise on construction or via
	///   the set location method.
	/// @sa SetPosition.
	Vec2 GetLocation() const noexcept { return m_location; }
	
	void SetLocation(const Vec2& value) noexcept { m_location = value; }

private:
	/// Linear position of the shape as initialized on construction or as assigned using the SetPosition method.
	Vec2 m_location = Vec2_zero;
};

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const CircleShape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const CircleShape& shape, const Transformation& xf, const Vec2& p);

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @note Behavior is undefined if the given density is negative.
/// @param density Density in kilograms per meter squared (must be non-negative).
/// @return Mass data for this shape.
MassData ComputeMass(const CircleShape& shape, float_t density);

} // namespace box2d

#endif
