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

#include <PlayRho/Collision/Shapes/ChainShape.hpp>

using namespace playrho;

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
    const auto count = conf.vertices.size();
    if (count > MaxChildCount)
    {
        throw InvalidArgument("too many vertices");
    }

    m_vertices = conf.vertices;
    
    if (count > 1)
    {
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
}

MassData ChainShape::GetMassData() const noexcept
{
    const auto density = GetDensity();
    if (density > Density(0))
    {
        const auto vertexCount = GetVertexCount();
        if (vertexCount > 1)
        {
            // XXX: This overcounts for the overlapping circle shape.
            auto mass = Mass{0};
            auto I = RotInertia{0};
            auto area = Area(0);
            auto center = Length2D{};
            const auto vertexRadius = GetVertexRadius();
            auto vprev = GetVertex(0);
            const auto circle_area = Square(vertexRadius) * Pi;
            for (auto i = decltype(vertexCount){1}; i < vertexCount; ++i)
            {
                const auto v = GetVertex(i);
                const auto massData = ::GetMassData(vertexRadius, density, vprev, v);
                mass += Mass{massData.mass};
                center += Real{Mass{massData.mass} / Kilogram} * massData.center;
                I += RotInertia{massData.I};
                
                const auto d = v - vprev;
                const auto b = GetLength(d);
                const auto h = vertexRadius * Real{2};
                area += b * h + circle_area;

                vprev = v;
            }
            center /= StripUnit(area);
            return MassData{center, mass, I};
        }
        if (vertexCount == 1)
        {
            return ::GetMassData(GetVertexRadius(), density, GetVertex(0));
        }
    }
    return MassData{};
}

ChildCounter ChainShape::GetChildCount() const noexcept
{
    // edge count = vertex count - 1
    const auto count = GetVertexCount();
    return (count > 1)? count - 1: count;
}

DistanceProxy ChainShape::GetChild(ChildCounter index) const
{
    if (index >= GetChildCount())
    {
        throw InvalidArgument("index out of range");
    }
    const auto vertexCount = GetVertexCount();
    if (vertexCount > 1)
    {
        return DistanceProxy{GetVertexRadius(), 2, &m_vertices[index], &m_normals[index * 2]};
    }
    return DistanceProxy{GetVertexRadius(), 1, &m_vertices[0], nullptr};
}
