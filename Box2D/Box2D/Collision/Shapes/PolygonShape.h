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

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.h>
#include <type_traits>

namespace box2d {

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to MaxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class PolygonShape : public Shape
{
public:
	/// Vertex count type.
	using vertex_count_t = std::remove_const<decltype(MaxPolygonVertices)>::type;

	PolygonShape(): Shape{e_polygon, PolygonRadius} {}

	PolygonShape(const PolygonShape&) = default;

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

	/// Gets the vertex count.
	/// @return value between 0 and MaxPolygonVertices inclusive.
	/// @see MaxPolygonVertices.
	vertex_count_t GetVertexCount() const noexcept { return m_count; }

	/// Get a vertex by index.
	Vec2 GetVertex(vertex_count_t index) const;

	/// Gets a normal by index.
	/// @detail
	/// These are 90-degree clockwise-rotated unit-vectors of the vectors defined by
	/// consecutive pairs of elements of vertices.
	/// @param index Index of the normal to get.
	/// @return Normal for the given index.
	Vec2 GetNormal(vertex_count_t index) const;

	const Vec2* GetVertices() const noexcept { return m_vertices; }

	const Vec2* GetNormals() const noexcept { return m_normals; }
	
	Vec2 GetCentroid() const noexcept { return m_centroid; }

	/// Validate convexity. This is a very time consuming operation.
	/// @returns true if valid
	bool Validate() const;
	
private:
	/// Array of vertices.
	/// @detail Consecutive vertices constitute "edges" of the polygon.
	/// @note This is some 16 x 8-bytes or 128-bytes large (on at least one platform).
	Vec2 m_vertices[MaxPolygonVertices];

	/// Normals of edges.
	/// @detail
	/// These are 90-degree clockwise-rotated unit-vectors of the vectors defined by
	/// consecutive pairs of elements of vertices.
	/// @note This is some 16 x 8-bytes or 128-bytes large (on at least one platform).
	Vec2 m_normals[MaxPolygonVertices];

	Vec2 m_centroid = Vec2_zero;
	
	vertex_count_t m_count = 0;
};

inline Vec2 PolygonShape::GetVertex(vertex_count_t index) const
{
	assert(0 <= index && index < m_count);
	return m_vertices[index];
}

inline Vec2 PolygonShape::GetNormal(vertex_count_t index) const
{
	assert(0 <= index && index < m_count);
	return m_normals[index];
}

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const PolygonShape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const PolygonShape& shape, const Transformation& xf, const Vec2& p);

/// Cast a ray against a child shape.
/// @param output the ray-cast results.
/// @param input the ray-cast input parameters.
/// @param transform the transform to be applied to the shape.
/// @param childIndex the child shape index
bool RayCast(const PolygonShape& shape, RayCastOutput* output, const RayCastInput& input,
			 const Transformation& transform, child_count_t childIndex);

/// Given a transform, compute the associated axis aligned bounding box for a child shape.
/// @param xf the world transform of the shape.
/// @param childIndex the child shape
/// @return the axis aligned box.
AABB ComputeAABB(const PolygonShape& shape, const Transformation& xf, child_count_t childIndex);

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @note Behavior is undefined if the given density is negative.
/// @param density Density in kilograms per meter squared (must be non-negative).
/// @return Mass data for this shape.
MassData ComputeMass(const PolygonShape& shape, float_t density);

} // namespace box2d
#endif
