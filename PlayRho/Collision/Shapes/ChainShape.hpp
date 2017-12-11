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

#ifndef PLAYRHO_COLLISION_SHAPES_CHAINSHAPE_HPP
#define PLAYRHO_COLLISION_SHAPES_CHAINSHAPE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/Shapes/ShapeDef.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/MassData.hpp>
#include <vector>

namespace playrho {

/// @brief Chain shape configuration.
///
/// @details A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated on the memory heap.
///
/// @image html Chain1.png
///
/// @warning The chain will not collide properly if there are self-intersections.
///
/// @ingroup PartsGroup
///
class ChainShapeConf: public ShapeDefBuilder<ChainShapeConf>
{
public:
    /// @brief Gets the default vertex radius.
    static PLAYRHO_CONSTEXPR inline NonNegative<Length> GetDefaultVertexRadius() noexcept
    {
        return DefaultLinearSlop * 2;
    }

    /// @brief Default constructor.
    ChainShapeConf();
    
    /// @brief Sets the configuration up for representing a chain of the given vertices.
    ChainShapeConf& Set(std::vector<Length2> vertices);

    /// @brief Adds the given vertex.
    ChainShapeConf& Add(Length2 vertex);

    /// @brief Gets the "child" shape count.
    ChildCounter GetChildCount() const noexcept
    {
        // edge count = vertex count - 1
        const auto count = GetVertexCount();
        return (count > 1)? count - 1: count;
    }

    /// @brief Gets the "child" shape at the given index.
    DistanceProxy GetChild(ChildCounter index) const;
    
    /// @brief Gets the mass data.
    MassData GetMassData() const noexcept;
    
    /// @brief Gets the vertex count.
    ChildCounter GetVertexCount() const noexcept
    {
        return static_cast<ChildCounter>(m_vertices.size());
    }
    
    /// @brief Gets a vertex by index.
    Length2 GetVertex(ChildCounter index) const
    {
        assert((0 <= index) && (index < GetVertexCount()));
        return m_vertices[index];
    }
    
    /// @brief Gets the normal at the given index.
    UnitVec2 GetNormal(ChildCounter index) const
    {
        assert((0 <= index) && (index < GetVertexCount()));
        return m_normals[index];
    }
    
    /// @brief Equality operator.
    friend bool operator== (const ChainShapeConf& lhs, const ChainShapeConf& rhs) noexcept
    {
        // Only need to check vertices are same since normals are calculated based on them.
        return lhs.m_vertices == rhs.m_vertices;
    }
    
    /// @brief Inequality operator.
    friend bool operator!= (const ChainShapeConf& lhs, const ChainShapeConf& rhs) noexcept
    {
        return !(lhs == rhs);
    }
    
private:
    std::vector<Length2> m_vertices; ///< Vertices.
    std::vector<UnitVec2> m_normals; ///< Normals.
};

// Free functions...

/// @brief Gets the child count for a given chain shape configuration.
inline ChildCounter GetChildCount(const ChainShapeConf& arg) noexcept
{
    return arg.GetChildCount();
}

/// @brief Gets the "child" shape for a given chain shape configuration.
inline DistanceProxy GetChild(const ChainShapeConf& arg, ChildCounter index)
{
    return arg.GetChild(index);
}

/// @brief Gets the mass data for a given chain shape configuration.
inline MassData GetMassData(const ChainShapeConf& arg) noexcept
{
    return arg.GetMassData();
}

/// @brief Determines whether the given shape is looped.
inline bool IsLooped(const ChainShapeConf& shape) noexcept
{
    const auto count = shape.GetVertexCount();
    return (count > 1)? (shape.GetVertex(count - 1) == shape.GetVertex(0)): false;
}

/// @brief Gets the next index after the given index for the given shape.
inline ChildCounter GetNextIndex(const ChainShapeConf& shape, ChildCounter index) noexcept
{
    return GetModuloNext(index, shape.GetVertexCount());
}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_CHAINSHAPE_HPP
