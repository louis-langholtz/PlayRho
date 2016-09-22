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

#ifndef B2_SHAPE_H
#define B2_SHAPE_H

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/AABB.hpp>
#include <Box2D/Collision/MassData.hpp>
#include <memory>

namespace box2d {

/// Base class for shapes.
/// @detail A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in World are created automatically when a Fixture
/// is created. Shapes may encapsulate a one or more child shapes.
class Shape
{
public:
	enum Type
	{
		e_circle = 0,
		e_edge = 1,
		e_polygon = 2,
		e_chain = 3,
		e_typeCount = 4
	};

	Shape() = delete;

	/// Initializing constructor.
	/// @param type Type of this shape object.
	/// @param radius Non-negative "radius" distance of this object (whose meaning is
	///   class dependent).
	/// @note Behavior is undefined if a negative radius is given.
	constexpr Shape(Type type, float_t radius) noexcept: m_type{type}, m_radius{radius}
	{
		assert(type < e_typeCount);
		assert(radius >= 0);
	}

	Shape(const Shape&) = default;

	~Shape() = default;

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	Type GetType() const noexcept { return m_type; }

	/// Gets the "radius" of the shape.
	/// @return a non-negative distance whose meaning is dependent on the object's class.
	float_t GetRadius() const noexcept { return m_radius; }
	
	void SetRadius(float_t radius) noexcept
	{
		assert(radius >= 0);
		m_radius = radius;
	}
	
private:
	const Type m_type;
	float_t m_radius;
};

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const Shape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const Shape& shape, const Transformation& xf, const Vec2& p);

/// Casts a ray against a child shape.
/// @param output the ray-cast results.
/// @param input the ray-cast input parameters.
/// @param xf Transform to be applied to the shape.
/// @param childIndex the child shape index
bool RayCast(const Shape& shape, RayCastOutput* output, const RayCastInput& input,
					 const Transformation& xf, child_count_t childIndex);

/// Given a transform, compute the associated axis aligned bounding box for a child shape.
/// @param xf the world transform of the shape.
/// @param childIndex the child shape
/// @return the axis aligned box.
AABB ComputeAABB(const Shape& shape, const Transformation& xf, child_count_t childIndex);

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @note Behavior is undefined if the given density is negative.
/// @param density Density in kilograms per meter squared (must be non-negative).
/// @return Mass data for this shape.
MassData ComputeMass(const Shape& shape, float_t density);
	
} // namespace box2d

#endif
