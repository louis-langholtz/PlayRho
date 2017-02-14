/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef B2_EDGE_SHAPE_H
#define B2_EDGE_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.hpp>

namespace box2d {

/// Edge shape.
/// @detail
/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
/// @note This data structure is 32-bytes.
class EdgeShape : public Shape
{
public:
	static constexpr RealNum GetDefaultVertexRadius() noexcept
	{
		return DefaultLinearSlop * 2;
	}

	EdgeShape(RealNum vertexRadius = GetDefaultVertexRadius()) noexcept:
		Shape{e_edge, vertexRadius}
	{
		// Intentionally empty.
	}

	constexpr EdgeShape(Vec2 v1, Vec2 v2,
						Vec2 v0 = GetInvalid<Vec2>(), Vec2 v3 = GetInvalid<Vec2>(),
						RealNum vertexRadius = GetDefaultVertexRadius()) noexcept:
		Shape{e_edge, vertexRadius},
		m_vertex0{v0}, m_vertex1{v1}, m_vertex2{v2}, m_vertex3{v3}
	{
		// Intentionally empty.
	}

	EdgeShape(const EdgeShape&) = default;

	/// Set this as an isolated edge.
	void Set(const Vec2 v1, const Vec2 v2);

	Vec2 GetVertex0() const noexcept { return m_vertex0; }
	Vec2 GetVertex1() const noexcept { return m_vertex1; }
	Vec2 GetVertex2() const noexcept { return m_vertex2; }
	Vec2 GetVertex3() const noexcept { return m_vertex3; }

	void SetVertex0(const Vec2 v) noexcept;
	void SetVertex3(const Vec2 v) noexcept;

	bool HasVertex0() const noexcept { return IsValid(m_vertex0); }
	bool HasVertex3() const noexcept { return IsValid(m_vertex3); }

private:
	/// These are the edge vertices
	Vec2 m_vertex1;
	Vec2 m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	Vec2 m_vertex0 = GetInvalid<Vec2>();
	Vec2 m_vertex3 = GetInvalid<Vec2>();
};

inline void EdgeShape::SetVertex0(const Vec2 v) noexcept
{
	m_vertex0 = v;
}

inline void EdgeShape::SetVertex3(const Vec2 v) noexcept
{
	m_vertex3 = v;
}

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const EdgeShape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const EdgeShape& shape, const Transformation& xf, const Vec2 p);

} // namespace box2d

#endif
