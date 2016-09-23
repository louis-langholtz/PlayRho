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

#include <Box2D/Collision/Shapes/Shape.h>

namespace box2d {

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
class EdgeShape : public Shape
{
public:
	EdgeShape(): Shape{e_edge} {}

	constexpr EdgeShape(const Vec2& v1, const Vec2& v2):
		Shape{e_edge}, m_vertex1{v1}, m_vertex2{v2} {}

	EdgeShape(const EdgeShape&) = default;

	/// Set this as an isolated edge.
	void Set(const Vec2& v1, const Vec2& v2);

	Vec2 GetVertex0() const noexcept { return m_vertex0; }
	Vec2 GetVertex1() const noexcept { return m_vertex1; }
	Vec2 GetVertex2() const noexcept { return m_vertex2; }
	Vec2 GetVertex3() const noexcept { return m_vertex3; }

	void SetVertex0(const Vec2& v) noexcept;
	void SetVertex3(const Vec2& v) noexcept;

	bool HasVertex0() const noexcept { return IsValid(m_vertex0); }
	bool HasVertex3() const noexcept { return IsValid(m_vertex3); }

private:
	/// These are the edge vertices
	Vec2 m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	Vec2 m_vertex0 = Vec2_invalid;
	Vec2 m_vertex3 = Vec2_invalid;
};

inline void EdgeShape::SetVertex0(const Vec2& v) noexcept
{
	m_vertex0 = v;
}

inline void EdgeShape::SetVertex3(const Vec2& v) noexcept
{
	m_vertex3 = v;
}
	
/// Gets the "radius" of the given shape.
float_t GetRadius(const EdgeShape& shape);

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const EdgeShape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const EdgeShape& shape, const Transformation& xf, const Vec2& p);

/// Cast a ray against a child shape.
/// @param input the ray-cast input parameters.
/// @param transform the transform to be applied to the shape.
/// @param childIndex the child shape index
RayCastOutput RayCast(const EdgeShape& shape, const RayCastInput& input,
					  const Transformation& transform, child_count_t childIndex);

/// Given a transform, compute the associated axis aligned bounding box for a child shape.
/// @param xf the world transform of the shape.
/// @param childIndex the child shape
/// @return the axis aligned box.
AABB ComputeAABB(const EdgeShape& shape, const Transformation& xf, child_count_t childIndex);

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @note Behavior is undefined if the given density is negative.
/// @param density Density in kilograms per meter squared (must be non-negative).
/// @return Mass data for this shape.
MassData ComputeMass(const EdgeShape& shape, float_t density);

} // namespace box2d

#endif
