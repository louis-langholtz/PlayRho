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

class EdgeShape;

/// @brief Chain shape.
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
class ChainShape
{
public:

    /// @brief Gets the default vertex radius.
    static PLAYRHO_CONSTEXPR inline NonNegative<Length> GetDefaultVertexRadius() noexcept
    {
        return DefaultLinearSlop * 2;
    }
    
    /// @brief Configuration data for chain shapes.
    class Conf: public ShapeDefBuilder<Conf>
    {
    public:
        Conf(): ShapeDefBuilder{ShapeDef{ShapeConf{}.UseVertexRadius(GetDefaultVertexRadius())}}
        {
            // Intentionally empty.
        }
        
        Conf& Set(std::vector<Length2> vertices);
        Conf& Add(Length2 vertex);

        ChildCounter GetChildCount() const noexcept
        {
            // edge count = vertex count - 1
            const auto count = GetVertexCount();
            return (count > 1)? count - 1: count;
        }

        DistanceProxy GetChild(ChildCounter index) const;
        
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

    private:
        std::vector<Length2> m_vertices; ///< Vertices.
        std::vector<UnitVec2> m_normals; ///< Normals.
    };

    /// @brief Gets the default configuration.
    static Conf GetDefaultConf() noexcept
    {
        return Conf{};
    }
};

// Free functions...

inline ChildCounter GetChildCount(const ChainShape::Conf& arg) noexcept
{
    return arg.GetChildCount();
}

inline DistanceProxy GetChild(const ChainShape::Conf& arg, ChildCounter index)
{
    return arg.GetChild(index);
}

inline MassData GetMassData(const ChainShape::Conf& arg) noexcept
{
    return arg.GetMassData();
}

/// @brief Determines whether the given shape is looped.
inline bool IsLooped(const ChainShape::Conf& shape) noexcept
{
    const auto count = shape.GetVertexCount();
    return (count > 1)? (shape.GetVertex(count - 1) == shape.GetVertex(0)): false;
}

/// @brief Gets the next index after the given index for the given shape.
inline ChildCounter GetNextIndex(const ChainShape::Conf& shape, ChildCounter index) noexcept
{
    return GetModuloNext(index, shape.GetVertexCount());
}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_CHAINSHAPE_HPP
