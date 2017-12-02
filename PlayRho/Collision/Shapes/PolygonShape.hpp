/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COLLISION_SHAPES_POLYGONSHAPE_HPP
#define PLAYRHO_COLLISION_SHAPES_POLYGONSHAPE_HPP

#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Collision/Shapes/ShapeDef.hpp>
#include <PlayRho/Common/VertexSet.hpp>
#include <type_traits>
#include <vector>

namespace playrho {

/// @brief Polygon shape.
/// @details A convex polygon. The interior of the polygon is to the left of each edge.
///   Polygons have a maximum number of vertices equal to MaxShapeVertices.
///   In most cases you should not need many vertices for a convex polygon.
/// @image html convex_concave.gif
/// @note This data structure is 64-bytes large (with 4-byte Real).
/// @ingroup PartsGroup
class PolygonShape : public Shape
{
public:

    /// Vertex count type.
    ///
    /// @note This type must not support more than 255 vertices as that would conflict
    ///   with the <code>ContactFeature::Index</code> type.
    ///
    using VertexCounter = std::remove_const<decltype(MaxShapeVertices)>::type;

    /// @brief Invalid vertex.
    static PLAYRHO_CONSTEXPR const auto InvalidVertex = static_cast<VertexCounter>(-1);

    /// @brief Gets the default vertex radius for the PolygonShape.
    static PLAYRHO_CONSTEXPR inline Length GetDefaultVertexRadius() noexcept
    {
        return DefaultLinearSlop * Real{2};
    }

    /// @brief Configuration data for polygon shapes.
    struct Conf: public ShapeDefBuilder<Conf>
    {
        PLAYRHO_CONSTEXPR inline Conf(): ShapeDefBuilder{ShapeConf{}.UseVertexRadius(GetDefaultVertexRadius())}
        {
            // Intentionally empty.
        }
    };
    
    /// @brief Gets the default configuration for a PolygonShape.
    static PLAYRHO_CONSTEXPR inline Conf GetDefaultConf() noexcept
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

    /// @brief Copy constructor.
    PolygonShape(const PolygonShape& other) = default;
    
    /// @brief Move constructor.
    PolygonShape(PolygonShape&& other) = default;
    
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
    explicit PolygonShape(Span<const Length2> points, const Conf& conf = GetDefaultConf()) noexcept;
    
    ~PolygonShape() override = default;
    
    /// @brief Copy assignment operator.
    PolygonShape& operator= (const PolygonShape& other) = default;
    
    /// @brief Move assignment operator.
    PolygonShape& operator= (PolygonShape&& other) = default;
    
    ChildCounter GetChildCount() const noexcept override;

    DistanceProxy GetChild(ChildCounter index) const override;
    
    MassData GetMassData() const noexcept override;
    
    void Accept(ShapeVisitor& visitor) const override;

    /// Creates a convex hull from the given array of local points.
    /// The size of the span must be in the range [1, MaxShapeVertices].
    /// @warning the points may be re-ordered, even if they form a convex polygon
    /// @warning collinear points are handled but not removed. Collinear points
    /// may lead to poor stacking behavior.
    void Set(Span<const Length2> points) noexcept;

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
    
    /// @brief Transforms this polygon by the given transformation.
    PolygonShape& Transform(Transformation xfm) noexcept;

    /// Gets the vertex count.
    /// @return value between 0 and MaxShapeVertices inclusive.
    /// @see MaxShapeVertices.
    VertexCounter GetVertexCount() const noexcept;

    /// Gets a vertex by index.
    /// @details Vertices go counter-clockwise.
    Length2 GetVertex(VertexCounter index) const;

    /// Gets a normal by index.
    /// @details
    /// These are 90-degree clockwise-rotated (outward-facing) unit-vectors of the edges defined
    /// by consecutive pairs of vertices starting with vertex 0.
    /// @param index Index of the normal to get.
    /// @return Normal for the given index.
    UnitVec2 GetNormal(VertexCounter index) const;

    /// Gets the span of vertices.
    /// @details Vertices go counter-clockwise.
    Span<const Length2> GetVertices() const noexcept
    {
        return Span<const Length2>(&m_vertices[0], GetVertexCount());
    }

    /// @brief Gets the span of normals.
    Span<const UnitVec2> GetNormals() const noexcept
    {
        return Span<const UnitVec2>(&m_normals[0], GetVertexCount());
    }
    
    /// @brief Gets the centroid.
    Length2 GetCentroid() const noexcept { return m_centroid; }
    
private:
    /// Array of vertices.
    /// @details Consecutive vertices constitute "edges" of the polygon.
    std::vector<Length2> m_vertices;

    /// Normals of edges.
    /// @details
    /// These are 90-degree clockwise-rotated unit-vectors of the vectors defined by
    /// consecutive pairs of elements of vertices.
    std::vector<UnitVec2> m_normals;

    /// Centroid of this shape.
    Length2 m_centroid = Length2{};
};

inline ChildCounter PolygonShape::GetChildCount() const noexcept
{
    return 1;
}

inline DistanceProxy PolygonShape::GetChild(ChildCounter index) const
{
    if (index != 0)
    {
        throw InvalidArgument("only index of 0 is supported");
    }
    return DistanceProxy{GetVertexRadius(),
        static_cast<DistanceProxy::size_type>(m_vertices.size()), m_vertices.data(),
        m_normals.data()};
}

inline PolygonShape::VertexCounter PolygonShape::GetVertexCount() const noexcept
{
    return static_cast<VertexCounter>(m_vertices.size());
}

inline Length2 PolygonShape::GetVertex(VertexCounter index) const
{
    assert(0 <= index && index < GetVertexCount());
    return m_vertices[index];
}

inline UnitVec2 PolygonShape::GetNormal(VertexCounter index) const
{
    assert(0 <= index && index < GetVertexCount());
    return m_normals[index];
}

// Free functions...

/// Gets the identified edge of the given polygon shape.
/// @note This must not be called for shapes with less than 2 vertices.
/// @warning Behavior is undefined if called for a shape with less than 2 vertices.
/// @relatedalso PolygonShape
Length2 GetEdge(const PolygonShape& shape, PolygonShape::VertexCounter index);

/// Validate convexity of the given shape.
/// @note This is a time consuming operation.
/// @returns true if valid
/// @relatedalso PolygonShape
bool Validate(const PolygonShape& shape);

/// Build vertices to represent an oriented box.
/// @param shape Shape to set as a box.
/// @param hx the half-width.
/// @param hy the half-height.
/// @param center the center of the box in local coordinates.
/// @param angle the rotation of the box in local coordinates.
/// @relatedalso PolygonShape
void SetAsBox(PolygonShape& shape, Length hx, Length hy, Length2 center, Angle angle) noexcept;

/// @brief Transforms the given shape by the given transformation.
/// @relatedalso PolygonShape
inline PolygonShape Transform(PolygonShape value, Transformation xfm) noexcept
{
    return value.Transform(xfm);
}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_POLYGONSHAPE_HPP
