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

#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Collision/Shapes/ShapeDef.hpp>

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
class EdgeShape : public Shape
{
public:
    
    /// @brief Gets the default vertex radius.
    static PLAYRHO_CONSTEXPR inline Length GetDefaultVertexRadius() noexcept
    {
        return DefaultLinearSlop * Real{2};
    }

    /// @brief Configuration data for edge shapes.
    struct Conf: public ShapeDefBuilder<Conf>
    {
        PLAYRHO_CONSTEXPR inline Conf(): ShapeDefBuilder{ShapeConf{}.UseVertexRadius(GetDefaultVertexRadius())}
        {
            // Intentionally empty.
        }
        
        /// @brief Uses the given value for vertex 1.
        PLAYRHO_CONSTEXPR inline Conf& UseVertex1(Length2 value) noexcept
        {
            vertex1 = value;
            return *this;
        }

        /// @brief Uses the given value for vertex 2.
        PLAYRHO_CONSTEXPR inline Conf& UseVertex2(Length2 value) noexcept
        {
            vertex2 = value;
            return *this;
        }

        Length2 vertex1 = Length2{}; ///< Vertex 1.
        Length2 vertex2 = Length2{}; ///< Vertex 2.
    };
    
    /// @brief Gets the default configuration for an EdgeShape.
    static PLAYRHO_CONSTEXPR inline Conf GetDefaultConf() noexcept
    {
        return Conf{};
    }

    /// @brief Initializing constructor.
    explicit EdgeShape(const Conf& conf = GetDefaultConf()) noexcept:
        Shape{conf},
        m_vertices{conf.vertex1, conf.vertex2}
    {
        m_normals[0] = GetUnitVector(GetFwdPerpendicular(conf.vertex2 - conf.vertex1));
        m_normals[1] = -m_normals[0];
    }

    /// @brief Initializing constructor.
    EdgeShape(Length2 v1, Length2 v2, const Conf& conf = GetDefaultConf()) noexcept:
        Shape{conf},
        m_vertices{v1, v2}
    {
        m_normals[0] = GetUnitVector(GetFwdPerpendicular(v2 - v1));
        m_normals[1] = -m_normals[0];
    }

    /// @brief Copy constructor.
    EdgeShape(const EdgeShape& other) = default;
    
    /// @brief Move constructor.
    EdgeShape(EdgeShape&& other) = default;
    
    ~EdgeShape() override = default;
    
    /// @brief Copy assignment operator.
    EdgeShape& operator= (const EdgeShape& other) = default;
    
    /// @brief Move assignment operator.
    EdgeShape& operator= (EdgeShape&& other) = default;

    ChildCounter GetChildCount() const noexcept override;

    DistanceProxy GetChild(ChildCounter index) const override;

    MassData GetMassData() const noexcept override;
    
    void Accept(ShapeVisitor& visitor) const override;

    /// @brief Sets this as an isolated edge.
    void Set(Length2 v1, Length2 v2);

    /// @brief Gets vertex number 1 (of 2).
    Length2 GetVertex1() const noexcept { return m_vertices[0]; }

    /// @brief Gets vertex number 2 (of 2).
    Length2 GetVertex2() const noexcept { return m_vertices[1]; }

    /// @brief Gets normal number 1 (of 2).
    UnitVec2 GetNormal1() const noexcept { return m_normals[0]; }

    /// @brief Gets normal number 2 (of 2).
    UnitVec2 GetNormal2() const noexcept { return m_normals[1]; }

private:
    Length2 m_vertices[2]; ///< Vertices
    UnitVec2 m_normals[2]; ///< Normals.
};

inline ChildCounter EdgeShape::GetChildCount() const noexcept
{
    return 1;
}

inline DistanceProxy EdgeShape::GetChild(ChildCounter index) const
{
    if (index != 0)
    {
        throw InvalidArgument("only index of 0 is supported");
    }
    return DistanceProxy{GetVertexRadius(), 2, m_vertices, m_normals};
}

} // namespace playrho

#endif // PLAYRHO_COLLISION_SHAPES_EDGESHAPE_HPP
