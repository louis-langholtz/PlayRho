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

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class b2PolygonShape : public b2Shape
{
public:
	using vertex_count_t = std::remove_cv<decltype(b2_maxPolygonVertices)>::type;

	b2PolygonShape(): b2Shape(e_polygon, b2_polygonRadius) {}

	b2PolygonShape(const b2PolygonShape&) = default;
	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const override;

	/// @see b2Shape::GetChildCount
	child_count_t GetChildCount() const override;

	/// Create a convex hull from the given array of local points.
	/// The count must be in the range [3, b2_maxPolygonVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	void Set(const b2Vec2 points[], vertex_count_t count);

	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(b2Float hx, b2Float hy) noexcept;

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	void SetAsBox(b2Float hx, b2Float hy, const b2Vec2& center, b2Float angle);

	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const override;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
					const b2Transform& transform, child_count_t childIndex) const override;

	/// @see b2Shape::ComputeAABB
	b2AABB ComputeAABB(const b2Transform& transform, child_count_t childIndex) const override;

	/// @see b2Shape::ComputeMass
	b2MassData ComputeMass(b2Float density) const override;

	/// Get the vertex count.
	vertex_count_t GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index.
	const b2Vec2& GetVertex(vertex_count_t index) const;

	/// Get a normal by index.
	const b2Vec2& GetNormal(vertex_count_t index) const;

	const b2Vec2* GetVertices() const noexcept { return m_vertices; }

	const b2Vec2* GetNormals() const noexcept { return m_normals; }
	
	b2Vec2 GetCentroid() const noexcept { return m_centroid; }

	/// Validate convexity. This is a very time consuming operation.
	/// @returns true if valid
	bool Validate() const;

private:
	b2Vec2 m_centroid = b2Vec2_zero;
	b2Vec2 m_vertices[b2_maxPolygonVertices];
	b2Vec2 m_normals[b2_maxPolygonVertices];
	vertex_count_t m_count = 0;
};

inline const b2Vec2& b2PolygonShape::GetVertex(vertex_count_t index) const
{
	b2Assert(0 <= index && index < m_count);
	return m_vertices[index];
}

inline const b2Vec2& b2PolygonShape::GetNormal(vertex_count_t index) const
{
	b2Assert(0 <= index && index < m_count);
	return m_normals[index];
}
#endif
