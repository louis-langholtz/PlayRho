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

#ifndef B2_EDGE_SHAPE_H
#define B2_EDGE_SHAPE_H

#include <PlayRho/Collision/Shapes/Shape.hpp>

namespace box2d {

/// Edge shape.
/// @details
/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
/// @note This data structure is 32-bytes.
class EdgeShape : public Shape
{
public:
    static constexpr Length GetDefaultVertexRadius() noexcept
    {
        return DefaultLinearSlop * Real{2};
    }

    struct Conf: public Builder<Conf>
    {
        constexpr Conf(): Builder<Conf>{Builder<Conf>{}.UseVertexRadius(GetDefaultVertexRadius())}
        {
            // Intentionally empty.
        }
        
        constexpr Conf& UseVertex1(Length2D value) noexcept
        {
            vertex1 = value;
            return *this;
        }
        constexpr Conf& UseVertex2(Length2D value) noexcept
        {
            vertex2 = value;
            return *this;
        }

        Length2D vertex1 = Length2D(0, 0);
        Length2D vertex2 = Length2D(0, 0);
    };
    
    static constexpr Conf GetDefaultConf() noexcept
    {
        return Conf{};
    }

    EdgeShape(const Conf& conf = GetDefaultConf()) noexcept:
        Shape{conf},
        m_vertices{conf.vertex1, conf.vertex2}
    {
        m_normals[0] = GetUnitVector(GetFwdPerpendicular(conf.vertex2 - conf.vertex1));
        m_normals[1] = -m_normals[0];
    }

    EdgeShape(Length2D v1, Length2D v2, const Conf& conf = GetDefaultConf()) noexcept:
        Shape{conf},
        m_vertices{v1, v2}
    {
        m_normals[0] = GetUnitVector(GetFwdPerpendicular(v2 - v1));
        m_normals[1] = -m_normals[0];
    }

    EdgeShape(const EdgeShape&) = default;

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

    /// Set this as an isolated edge.
    void Set(const Length2D v1, const Length2D v2);

    Length2D GetVertex1() const noexcept { return m_vertices[0]; }
    Length2D GetVertex2() const noexcept { return m_vertices[1]; }

    UnitVec2 GetNormal1() const noexcept { return m_normals[0]; }
    UnitVec2 GetNormal2() const noexcept { return m_normals[1]; }

private:
    /// These are the edge vertices
    Length2D m_vertices[2];
    UnitVec2 m_normals[2];
};

inline ChildCounter EdgeShape::GetChildCount() const noexcept
{
    return 1;
}

inline DistanceProxy EdgeShape::GetChild(ChildCounter index) const
{
    if (index != 0)
    {
        throw InvalidArgument("index out of range");
    }
    return (index == 0)?
        DistanceProxy{GetVertexRadius(), 2, m_vertices, m_normals}:
        DistanceProxy{};
}

inline void EdgeShape::Accept(box2d::Shape::Visitor &visitor) const
{
    visitor.Visit(*this);
}

} // namespace box2d

#endif
