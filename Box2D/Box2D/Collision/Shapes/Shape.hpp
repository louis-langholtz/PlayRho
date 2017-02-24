/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/Math.hpp>

namespace box2d {

class Fixture;

/// Base class for shapes.
/// @detail A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in World are created automatically when a Fixture
/// is created. Shapes may encapsulate one or more child shapes.
/// @note This data structure is 8-bytes large (on at least one 64-bit platform).
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
	constexpr Shape(Type type, RealNum vertexRadius) noexcept:
		m_type{type}, m_vertexRadius{vertexRadius}
	{
		assert(type < e_typeCount);
		assert(vertexRadius >= 0);
	}

	Shape(const Shape&) = default;

	~Shape() = default;

	/// Gets the type of this shape.
	/// @note You can use this to down cast to the concrete shape.
	/// @return the shape type.
	Type GetType() const noexcept { return m_type; }
	
	RealNum GetVertexRadius() const noexcept { return m_vertexRadius; }

	void SetVertexRadius(RealNum vertexRadius)
	{
		assert(vertexRadius > std::numeric_limits<RealNum>::min());
		m_vertexRadius = vertexRadius;
	}

private:
	Type m_type;
	RealNum m_vertexRadius;
};

/// Gets the vertex radius of the given shape (in meters).
/// @detail Gets the radius (in meters) of every vertex of this shape.
/// This is used for collision handling.
/// @note This value should never be less than zero.
/// @sa DistanceProxy.
inline RealNum GetVertexRadius(const Shape& shape)
{
	return shape.GetVertexRadius();
}

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const Shape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const Shape& shape, const Transformation& xf, const Vec2 p);

/// Determine if two generic shapes overlap.
bool TestOverlap(const Shape& shapeA, child_count_t indexA, const Transformation& xfA,
				 const Shape& shapeB, child_count_t indexB, const Transformation& xfB);

Shape::Type GetType(const Fixture& fixture) noexcept;
	
} // namespace box2d

#endif
