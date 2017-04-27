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

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Common/VertexSet.hpp>
#include <type_traits>
#include <vector>

namespace box2d {

/// Polygon shape.
/// @details
/// A convex polygon. The interior of the polygon is to the left of each edge.
/// Polygons have a maximum number of vertices equal to MaxShapeVertices.
/// In most cases you should not need many vertices for a convex polygon.
/// @note This data structure is 64-bytes large (with 4-byte RealNum).
class PolygonShape : public Shape
{
public:

	/// Vertex count type.
	///
	/// @note This type must not support more than 255 vertices as that would conflict
	///   with the <code>ContactFeature::index_t</code> type.
	///
	using vertex_count_t = std::uint8_t;

	static constexpr auto InvalidVertex = static_cast<vertex_count_t>(-1);

	static constexpr Length GetDefaultVertexRadius() noexcept
	{
		return DefaultLinearSlop * RealNum{2};
	}

	struct Conf: public Shape::Conf
	{
		constexpr Conf(): Shape::Conf{Shape::Conf{}.UseVertexRadius(GetDefaultVertexRadius())}
		{
		}
	};
	
	static constexpr Conf GetDefaultConf() noexcept
	{
		return Conf{};
	}
	
	/// Default constructor.
	/// @details Constructs a polygon shape with a 0,0 centroid and vertex count of 0.
	/// @note Polygons with a vertex count less than 1 are "degenerate" and should be
	///   treated as invalid.
	explicit PolygonShape(const Conf& conf = GetDefaultConf()) noexcept:
		Shape{conf}
	{
		// Intentionally empty.
	}

	PolygonShape(const PolygonShape&) = default;
	
	/// Initializing constructor for rectangles.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param conf Configuration data for the shape.
	explicit PolygonShape(Length hx, Length hy, const Conf& conf = GetDefaultConf()) noexcept;
	
	/// Creates a convex hull from the given array of local points.
	/// The size of the span must be in the range [1, MaxShapeVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	explicit PolygonShape(Span<const Length2D> points, const Conf& conf = GetDefaultConf()) noexcept;
	
	/// Gets the number of child primitives.
	/// @return Positive non-zero count.
	child_count_t GetChildCount() const noexcept override;

	DistanceProxy GetChild(child_count_t index) const noexcept override;

	/// Tests a point for containment in this shape.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	/// @return <code>true</code> if point is contained in this shape, <code>false</code> otherwise.
	bool TestPoint(const Transformation& xf, const Length2D p) const noexcept override;

	/// Computes the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @note Behavior is undefined if the density is negative.
	/// @return Mass data for this shape.
	MassData GetMassData() const noexcept override;

	/// Cast a ray against a child shape.
	/// @param input the ray-cast input parameters.
	/// @param xf the transform to be applied to the shape.
	/// @param childIndex the child shape index
	RayCastOutput RayCast(const RayCastInput& input, const Transformation& xf,
						  child_count_t childIndex) const noexcept override;

	void Accept(Visitor& visitor) const override;

	/// Creates a convex hull from the given array of local points.
	/// The size of the span must be in the range [1, MaxShapeVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	void Set(Span<const Length2D> points) noexcept;

	/// Creates a convex hull from the given set of local points.
	/// The size of the set must be in the range [1, MaxShapeVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	///   may lead to poor stacking behavior.
	void Set(const VertexSet& points) noexcept;
	
	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(Length hx, Length hy) noexcept;
	
	void Transform(Transformation xfm) noexcept;

	/// Gets the vertex count.
	/// @return value between 0 and MaxShapeVertices inclusive.
	/// @see MaxShapeVertices.
	vertex_count_t GetVertexCount() const noexcept;

	/// Gets a vertex by index.
	/// @details Vertices go counter-clockwise.
	Length2D GetVertex(vertex_count_t index) const;

	/// Gets a normal by index.
	/// @details
	/// These are 90-degree clockwise-rotated (outward-facing) unit-vectors of the edges defined
	/// by consecutive pairs of vertices starting with vertex 0.
	/// @param index Index of the normal to get.
	/// @return Normal for the given index.
	UnitVec2 GetNormal(vertex_count_t index) const;

	/// Gets the span of vertices.
	/// @details Vertices go counter-clockwise.
	Span<const Length2D> GetVertices() const noexcept
	{
		return Span<const Length2D>(&m_vertices[0], GetVertexCount());
	}

	Span<const UnitVec2> GetNormals() const noexcept
	{
		return Span<const UnitVec2>(&m_normals[0], GetVertexCount());
	}
	
	Length2D GetCentroid() const noexcept { return m_centroid; }
	
private:
	/// Array of vertices.
	/// @details Consecutive vertices constitute "edges" of the polygon.
	std::vector<Length2D> m_vertices;

	/// Normals of edges.
	/// @details
	/// These are 90-degree clockwise-rotated unit-vectors of the vectors defined by
	/// consecutive pairs of elements of vertices.
	std::vector<UnitVec2> m_normals;

	/// Centroid of this shape.
	Length2D m_centroid = Vec2_zero * Meter;
};

inline child_count_t PolygonShape::GetChildCount() const noexcept
{
	return 1;
}

inline DistanceProxy PolygonShape::GetChild(child_count_t index) const noexcept
{
	assert(index == 0);
	return (index == 0)?
		DistanceProxy{GetVertexRadius(), static_cast<DistanceProxy::size_type>(m_vertices.size()),
			&m_vertices[0], &m_normals[0]}:
		DistanceProxy{};
}

inline void PolygonShape::Accept(box2d::Shape::Visitor &visitor) const
{
	visitor.Visit(*this);
}

inline PolygonShape::vertex_count_t PolygonShape::GetVertexCount() const noexcept
{
	return static_cast<vertex_count_t>(m_vertices.size());
}

inline Length2D PolygonShape::GetVertex(vertex_count_t index) const
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
Length2D GetEdge(const PolygonShape& shape, PolygonShape::vertex_count_t index);

/// Validate convexity of the given shape.
/// @note This is a time consuming operation.
/// @returns true if valid
bool Validate(const PolygonShape& shape);

/// Build vertices to represent an oriented box.
/// @param shape Shape to set as a box.
/// @param hx the half-width.
/// @param hy the half-height.
/// @param center the center of the box in local coordinates.
/// @param angle the rotation of the box in local coordinates.
void SetAsBox(PolygonShape& shape, Length hx, Length hy, const Length2D center, Angle angle) noexcept;

inline PolygonShape Transform(PolygonShape value, Transformation xfm) noexcept
{
	value.Transform(xfm);
	return value;
}

} // namespace box2d
#endif
