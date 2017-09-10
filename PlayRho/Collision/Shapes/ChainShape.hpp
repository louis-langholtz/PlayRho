/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef PLAYRHO_CHAIN_SHAPE_HPP
#define PLAYRHO_CHAIN_SHAPE_HPP

#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <vector>

namespace playrho {

class EdgeShape;

/// @brief Chain shape.
///
/// @details A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated on the memory heap.
///
/// @warning The chain will not collide properly if there are self-intersections.
///
class ChainShape: public Shape
{
public:

    /// @brief Gets the default vertex radius.
    static constexpr Length GetDefaultVertexRadius() noexcept
    {
        return DefaultLinearSlop * Real{2};
    }
    
    /// @brief Configuration data for chain shapes.
    struct Conf: public Builder<Conf>
    {
        Conf(): Builder<Conf>{Builder<Conf>{}.UseVertexRadius(GetDefaultVertexRadius())}
        {
            // Intentionally empty.
        }
        
        std::vector<Length2D> vertices;
    };

    /// @brief Gets the default configuration.
    static Conf GetDefaultConf() noexcept
    {
        return Conf{};
    }
    
    /// @brief Initializing constructor.
    ChainShape(const Conf& conf = GetDefaultConf());

    /// @brief Copy constructor.
    ChainShape(const ChainShape& other) = default;

    virtual ~ChainShape() = default;

    /// Gets the number of child primitives.
    /// @return Positive non-zero count.
    ChildCounter GetChildCount() const noexcept override;
    
    /// @brief Gets the child for the given index.
    /// @throws InvalidArgument if the index is out of range.
    DistanceProxy GetChild(ChildCounter index) const override;

    /// Computes the mass properties of this shape using its dimensions and density.
    /// The inertia tensor is computed about the local origin.
    /// @return Mass data for this shape.
    MassData GetMassData() const noexcept override;
    
    void Accept(Visitor& visitor) const override;

    /// Get the vertex count.
    ChildCounter GetVertexCount() const noexcept;

    /// Get a vertex by index.
    Length2D GetVertex(ChildCounter index) const;

    /// @brief Gets the normal at the given index.
    UnitVec2 GetNormal(ChildCounter index) const;

private:
    std::vector<Length2D> m_vertices;
    std::vector<UnitVec2> m_normals;
};

inline ChildCounter ChainShape::GetVertexCount() const noexcept
{
    return static_cast<ChildCounter>(m_vertices.size());
}

inline void ChainShape::Accept(playrho::Shape::Visitor &visitor) const
{
    visitor.Visit(*this);
}

inline Length2D ChainShape::GetVertex(ChildCounter index) const
{
    assert((0 <= index) && (index < GetVertexCount()));
    return m_vertices[index];
}

inline UnitVec2 ChainShape::GetNormal(ChildCounter index) const
{
    assert((0 <= index) && (index < GetVertexCount()));
    return m_normals[index];
}

/// @brief Determines whether the given shape is looped.
inline bool IsLooped(const ChainShape& shape) noexcept
{
    const auto count = shape.GetVertexCount();
    return (count > 1)? (shape.GetVertex(count - 1) == shape.GetVertex(0)): false;
}

/// @brief Gets the next index after the given index for the given shape.
inline ChildCounter GetNextIndex(const ChainShape& shape, ChildCounter index) noexcept
{
    return GetModuloNext(index, shape.GetVertexCount());
}

} // namespace playrho

#endif
