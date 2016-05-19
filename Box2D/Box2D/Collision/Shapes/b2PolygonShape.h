/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include <Box2D/Collision/Shapes/b2Shape.h>
#include <type_traits>

namespace box2d {

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to MaxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class b2PolygonShape : public Shape
{
public:
	using vertex_count_t = std::remove_cv<decltype(MaxPolygonVertices)>::type;

	b2PolygonShape(): Shape(e_polygon, PolygonRadius) {}

	b2PolygonShape(const b2PolygonShape&) = default;
	/// Implement Shape.
	Shape* Clone(BlockAllocator* allocator) const override;

	/// @see Shape::GetChildCount
	child_count_t GetChildCount() const override;

	/// Create a convex hull from the given array of local points.
	/// The count must be in the range [3, MaxPolygonVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	void Set(const Vec2 points[], vertex_count_t count);

	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(float_t hx, float_t hy) noexcept;

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	void SetAsBox(float_t hx, float_t hy, const Vec2& center, float_t angle);

	/// @see Shape::TestPoint
	bool TestPoint(const Transform& transform, const Vec2& p) const override;

	/// Implement Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
					const Transform& transform, child_count_t childIndex) const override;

	/// @see Shape::ComputeAABB
	AABB ComputeAABB(const Transform& transform, child_count_t childIndex) const override;

	/// @see Shape::ComputeMass
	MassData ComputeMass(float_t density) const override;

	/// Gets the vertex count.
	/// @return value between 0 and MaxPolygonVertices inclusive.
	/// @see MaxPolygonVertices.
	vertex_count_t GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index.
	Vec2 GetVertex(vertex_count_t index) const;

	/// Get a normal by index.
	Vec2 GetNormal(vertex_count_t index) const;

	const Vec2* GetVertices() const noexcept { return m_vertices; }

	const Vec2* GetNormals() const noexcept { return m_normals; }
	
	Vec2 GetCentroid() const noexcept { return m_centroid; }

	/// Validate convexity. This is a very time consuming operation.
	/// @returns true if valid
	bool Validate() const;

private:
	Vec2 m_centroid = Vec2_zero;
	Vec2 m_vertices[MaxPolygonVertices];
	Vec2 m_normals[MaxPolygonVertices];
	vertex_count_t m_count = 0;
};

inline Vec2 b2PolygonShape::GetVertex(vertex_count_t index) const
{
	assert(0 <= index && index < m_count);
	return m_vertices[index];
}

inline Vec2 b2PolygonShape::GetNormal(vertex_count_t index) const
{
	assert(0 <= index && index < m_count);
	return m_normals[index];
}
	
} // namespace box2d
#endif
