/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Collision/Shapes/ChainShape.hpp>

using namespace box2d;

namespace {
#if 0
    inline bool IsEachVertexFarEnoughApart(Span<const Length2D> vertices)
    {
        for (auto i = decltype(vertices.size()){1}; i < vertices.size(); ++i)
        {
            const auto delta = vertices[i-1] - vertices[i];
            
            // XXX not quite right unit-wise but this works well enough.
            if (GetLengthSquared(GetVec2(delta)) * Meter <= DefaultLinearSlop)
            {
                return false;
            }
        }
        return true;
    }
#endif
} // anonymous namespace

ChainShape::ChainShape(const Conf& conf):
    Shape{conf}
{
    if (conf.vertices.size() > MaxChildCount)
    {
        throw InvalidArgument("too many vertices");
    }

    const auto count = static_cast<ChildCounter>(conf.vertices.size());
    m_count = count;
    m_vertices = conf.vertices;
    
    auto vprev = m_vertices[0];
    for (auto i = decltype(count){1}; i < count; ++i)
    {
        // Get the normal and push it and its reverse.
        // This "doubling up" of the normals, makes the GetChild() method work.
        const auto v = m_vertices[i];
        const auto normal = GetUnitVector(GetFwdPerpendicular(v - vprev));
        m_normals.push_back(normal);
        m_normals.push_back(-normal);
        vprev = v;
    }
}

MassData ChainShape::GetMassData() const noexcept
{
    const auto density = GetDensity();
    if (density > Density(0))
    {
        // XXX: This overcounts for the overlapping circle shape.
        auto mass = Mass{0};
        auto I = RotInertia{0};
        auto center = Length2D(0, 0);
        const auto vertexRadius = GetVertexRadius();
        const auto childCount = GetChildCount();
        auto vprev = GetVertex(0);
        for (auto i = decltype(childCount){1}; i < childCount; ++i)
        {
            const auto v = GetVertex(i);
            const auto massData = ::GetMassData(vertexRadius, density, vprev, v);
            mass += Mass{massData.mass};
            center += Real{Mass{massData.mass} / Kilogram} * massData.center;
            I += RotInertia{massData.I};
            vprev = v;
        }
        return MassData{mass, center, I};
    }
    return MassData{NonNegative<Mass>{0}, Length2D(0, 0), NonNegative<RotInertia>{0}};
}

ChildCounter ChainShape::GetChildCount() const noexcept
{
    // edge count = vertex count - 1
    const auto count = GetVertexCount();
    return (count > 1)? count - 1: 0;
}

DistanceProxy ChainShape::GetChild(ChildCounter index) const
{
    if (index >= GetVertexCount())
    {
        throw InvalidArgument("index out of range");
    }
    return DistanceProxy{GetVertexRadius(), 2, &m_vertices[index], &m_normals[index * 2]};
}
