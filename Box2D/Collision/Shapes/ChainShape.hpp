/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef B2_CHAIN_SHAPE_H
#define B2_CHAIN_SHAPE_H

#include <Box2D/Collision/Shapes/Shape.hpp>
#include <vector>

namespace box2d {

class EdgeShape;

/// Chain shape.
///
/// @details A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using alloc.
///
/// @warning The chain will not collide properly if there are self-intersections.
///
class ChainShape: public Shape
{
public:
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
    
    ChainShape(const Conf& conf = GetDefaultConf()):
        Shape{conf}
    {
        // Intentionally empty.
    }

    ChainShape(const ChainShape& other);

    /// The destructor frees the vertices using free.
    virtual ~ChainShape();

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
    /// @return Mass data for this shape.
    MassData GetMassData() const noexcept override;
    
    void Accept(Visitor& visitor) const override;

    ChainShape& operator=(const ChainShape& other);

    /// Clear all data.
    void Clear();

    /// Create a loop. This automatically adjusts connectivity.
    /// @note Behavior is undefined if vertices is null or if count of vertices is less than 3.
    /// @param vertices Non-null array of vertices. These are copied.
    void CreateLoop(Span<const Length2D> vertices);

    /// Create a chain with isolated end vertices.
    /// @param vertices an array of vertices, these are copied
    void CreateChain(Span<const Length2D> vertices);
    
    /// Get the vertex count.
    child_count_t GetVertexCount() const noexcept { return m_count; }

    /// Get a vertex by index.
    Length2D GetVertex(child_count_t index) const;

    UnitVec2 GetNormal(child_count_t index) const;

private:
    std::vector<Length2D> m_vertices;
    std::vector<UnitVec2> m_normals;

    /// The vertex count.
    child_count_t m_count = 0;
};

inline void ChainShape::Accept(box2d::Shape::Visitor &visitor) const
{
    visitor.Visit(*this);
}

inline Length2D ChainShape::GetVertex(child_count_t index) const
{
    assert((0 <= index) && (index < m_count));
    return m_vertices[index];
}

inline UnitVec2 ChainShape::GetNormal(child_count_t index) const
{
    assert((0 <= index) && (index < m_count));
    return m_normals[index];
}

inline bool IsLooped(const ChainShape& shape) noexcept
{
    const auto count = shape.GetVertexCount();
    return (count > 1)? (shape.GetVertex(count - 1) == shape.GetVertex(0)): false;
}

inline child_count_t GetNextIndex(const ChainShape& shape, child_count_t index) noexcept
{
    return GetModuloNext(index, shape.GetVertexCount());
}

} // namespace box2d

#endif
