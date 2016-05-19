/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/b2Shape.h>

namespace box2d {

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
class b2EdgeShape : public Shape
{
public:
	b2EdgeShape(): Shape(e_edge, PolygonRadius) {}

	constexpr b2EdgeShape(const Vec2& v1, const Vec2& v2):
		Shape(e_edge, PolygonRadius), m_vertex1(v1), m_vertex2(v2) {}

	b2EdgeShape(const b2EdgeShape&) = default;

	/// Set this as an isolated edge.
	void Set(const Vec2& v1, const Vec2& v2);

	/// Implement Shape.
	Shape* Clone(BlockAllocator* allocator) const override;

	/// @see Shape::GetChildCount
	child_count_t GetChildCount() const override;

	/// @see Shape::TestPoint
	bool TestPoint(const Transform& transform, const Vec2& p) const override;

	/// Implement Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
				const Transform& transform, child_count_t childIndex) const override;

	/// @see Shape::ComputeAABB
	AABB ComputeAABB(const Transform& transform, child_count_t childIndex) const override;

	/// @see Shape::ComputeMass
	b2MassData ComputeMass(float_t density) const override;

	Vec2 GetVertex0() const noexcept { return m_vertex0; }
	Vec2 GetVertex1() const noexcept { return m_vertex1; }
	Vec2 GetVertex2() const noexcept { return m_vertex2; }
	Vec2 GetVertex3() const noexcept { return m_vertex3; }

	void SetVertex0(const Vec2& v) noexcept;
	void SetVertex3(const Vec2& v) noexcept;

	bool HasVertex0() const noexcept { return m_hasVertex0; }
	bool HasVertex3() const noexcept { return m_hasVertex3; }

private:
	/// These are the edge vertices
	Vec2 m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	Vec2 m_vertex0 = Vec2_zero, m_vertex3 = Vec2_zero;
	bool m_hasVertex0 = false, m_hasVertex3 = false;
};

inline void b2EdgeShape::SetVertex0(const Vec2& v) noexcept
{
	m_vertex0 = v;
	m_hasVertex0 = true;
}

inline void b2EdgeShape::SetVertex3(const Vec2& v) noexcept
{
	m_vertex3 = v;
	m_hasVertex3 = true;
}
	
} // namespace box2d

#endif
