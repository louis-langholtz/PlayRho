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

#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Common/VertexSet.hpp>
#include <type_traits>
#include <vector>

namespace box2d {

/// Polygon shape.
/// @detail
/// A convex polygon. The interior of the polygon is to the left of each edge.
/// Polygons have a maximum number of vertices equal to MaxShapeVertices.
/// In most cases you should not need many vertices for a convex polygon.
/// @note This data structure is 64-bytes large (with 4-byte realnum).
class PolygonShape : public Shape
{
public:
	/// Vertex count type.
	using vertex_count_t = uint8;

	static constexpr auto InvalidVertex = static_cast<vertex_count_t>(-1);

	static constexpr realnum GetDefaultVertexRadius() noexcept
	{
		return LinearSlop * 2;
	}

	/// Default constructor.
	/// @detail Constructs a polygon shape with a 0,0 centroid and vertex count of 0.
	/// @note Polygons with a vertex count less than 1 are "degenerate" and should be
	///   treated as invalid.
	PolygonShape(realnum vertexRadius = GetDefaultVertexRadius()) noexcept:
		Shape{e_polygon, vertexRadius}
	{
		// Intentionally empty.
	}

	PolygonShape(const PolygonShape&) = default;
	
	/// Initializing constructor for rectangles.
	/// @param hx the half-width.
	/// @param hy the half-height.
	explicit PolygonShape(realnum hx, realnum hy) noexcept;
	
	/// Creates a convex hull from the given array of local points.
	/// The size of the span must be in the range [1, MaxShapeVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	PolygonShape(Span<const Vec2> points) noexcept;
	
	/// Creates a convex hull from the given array of local points.
	/// The size of the span must be in the range [1, MaxShapeVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	void Set(Span<const Vec2> points) noexcept;

	/// Creates a convex hull from the given set of local points.
	/// The size of the set must be in the range [1, MaxShapeVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	void Set(const VertexSet& points) noexcept;
	
	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(realnum hx, realnum hy) noexcept;
	
	void Transform(Transformation xfm) noexcept;

	/// Gets the vertex count.
	/// @return value between 0 and MaxShapeVertices inclusive.
	/// @see MaxShapeVertices.
	vertex_count_t GetVertexCount() const noexcept
	{
		return static_cast<vertex_count_t>(m_vertices.size());
	}

	/// Gets a vertex by index.
	/// @detail Vertices go counter-clockwise.
	Vec2 GetVertex(vertex_count_t index) const;

	/// Gets a normal by index.
	/// @detail
	/// These are 90-degree clockwise-rotated (outward-facing) unit-vectors of the edges defined
	/// by consecutive pairs of vertices starting with vertex 0.
	/// @param index Index of the normal to get.
	/// @return Normal for the given index.
	UnitVec2 GetNormal(vertex_count_t index) const;

	/// Gets the span of vertices.
	/// @detail Vertices go counter-clockwise.
	Span<const Vec2> GetVertices() const noexcept { return Span<const Vec2>(&m_vertices[0], GetVertexCount()); }

	Span<const UnitVec2> GetNormals() const noexcept { return Span<const UnitVec2>(&m_normals[0], GetVertexCount()); }
	
	Vec2 GetCentroid() const noexcept { return m_centroid; }
	
private:
	/// Array of vertices.
	/// @detail Consecutive vertices constitute "edges" of the polygon.
	std::vector<Vec2> m_vertices;

	/// Normals of edges.
	/// @detail
	/// These are 90-degree clockwise-rotated unit-vectors of the vectors defined by
	/// consecutive pairs of elements of vertices.
	std::vector<UnitVec2> m_normals;

	/// Centroid of this shape.
	Vec2 m_centroid = Vec2_zero;
};

inline Vec2 PolygonShape::GetVertex(vertex_count_t index) const
{
	assert(0 <= index && index < GetVertexCount());
	return m_vertices[index];
}

inline UnitVec2 PolygonShape::GetNormal(vertex_count_t index) const
{
	assert(0 <= index && index < GetVertexCount());
	return m_normals[index];
}

/// Gets the identified edge of the given polygon shape.
/// @note This must not be called for shapes with less than 2 vertices.
/// @warning Behavior is undefined if called for a shape with less than 2 vertices.
Vec2 GetEdge(const PolygonShape& shape, PolygonShape::vertex_count_t index);

/// Gets the number of child primitives.
/// @return Positive non-zero count.
child_count_t GetChildCount(const PolygonShape& shape);

/// Tests a point for containment in this shape.
/// @param xf the shape world transform.
/// @param p a point in world coordinates.
/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
bool TestPoint(const PolygonShape& shape, const Transformation& xf, const Vec2 p);

/// Validate convexity of the given shape.
/// @note This is a time consuming operation.
/// @returns true if valid
bool Validate(const PolygonShape& shape);

/// Build vertices to represent an oriented box.
/// @param hx the half-width.
/// @param hy the half-height.
/// @param center the center of the box in local coordinates.
/// @param angle the rotation of the box in local coordinates.
void SetAsBox(PolygonShape& shape, realnum hx, realnum hy, const Vec2 center, Angle angle) noexcept;
	
size_t FindLowestRightMostVertex(Span<const Vec2> vertices);
	
inline PolygonShape Transform(PolygonShape value, Transformation xfm) noexcept
{
	value.Transform(xfm);
	return value;
}

} // namespace box2d
#endif
