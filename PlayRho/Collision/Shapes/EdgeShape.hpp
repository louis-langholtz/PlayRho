/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_COLLISION_SHAPES_EDGESHAPE_HPP
#define PLAYRHO_COLLISION_SHAPES_EDGESHAPE_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/Shapes/ShapeDef.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/MassData.hpp>

namespace playrho {

/// @brief Edge shape.
///
/// @details A line segment (edge) shape. These can be connected in chains or loops
///   to other edge shapes. The connectivity information is used to ensure correct
///   contact normals.
///
/// @note This data structure is 56-bytes.
///
/// @ingroup PartsGroup
///
class EdgeShape
{
public:
    
    /// @brief Gets the default vertex radius.
    static PLAYRHO_CONSTEXPR inline Length GetDefaultVertexRadius() noexcept
    {
        return DefaultLinearSlop * Real{2};
    }

    /// @brief Configuration data for edge shapes.
    class Conf: public ShapeDefBuilder<Conf>
    {
    public:
        Conf(): ShapeDefBuilder{ShapeConf{}.UseVertexRadius(GetDefaultVertexRadius())}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        Conf(Length2 vA, Length2 vB, const Conf& conf = GetDefaultConf()) noexcept:
            ShapeDefBuilder{conf},
            m_vertices{vA, vB}
        {
            const auto normal = GetUnitVector(GetFwdPerpendicular(vB - vA));
            m_normals[0] = normal;
            m_normals[1] = -normal;
        }
        
        /// @brief Sets both vertices in one call.
        inline Conf& Set(Length2 vA, Length2 vB) noexcept
        {
            m_vertices[0] = vA;
            m_vertices[1] = vB;
            const auto normal = GetUnitVector(GetFwdPerpendicular(vB - vA));
            m_normals[0] = normal;
            m_normals[1] = -normal;
            return *this;
        }
        
        Length2 GetVertexA() const noexcept
        {
            return m_vertices[0];
        }
        
        Length2 GetVertexB() const noexcept
        {
            return m_vertices[1];
        }
        
        DistanceProxy GetChild() const noexcept
        {
            return DistanceProxy{vertexRadius, 2, m_vertices, m_normals};
        }
        
    private:
        Length2 m_vertices[2] = {Length2{}, Length2{}}; ///< Vertices
        UnitVec2 m_normals[2] = {UnitVec2{}, UnitVec2{}}; ///< Normals.
    };
    
    /// @brief Gets the default configuration for an EdgeShape.
    static inline Conf GetDefaultConf() noexcept
    {
        return Conf{};
    }
};

// Free functions...

PLAYRHO_CONSTEXPR inline ChildCounter GetChildCount(const EdgeShape::Conf&) noexcept
{
    return 1;
}

inline DistanceProxy GetChild(const EdgeShape::Conf& arg, ChildCounter index)
{
    if (index != 0)
    {
        throw InvalidArgument("only index of 0 is supported");
    }
    return arg.GetChild();
}

inline MassData GetMassData(const EdgeShape::Conf& arg) noexcept
{
    return playrho::GetMassData(arg.vertexRadius, arg.density,
                                arg.GetVertexA(), arg.GetVertexB());
}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_EDGESHAPE_HPP
