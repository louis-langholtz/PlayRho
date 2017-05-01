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

#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/RayCastInput.hpp>

#include <cstring>

using namespace box2d;

namespace {
    
    inline bool IsEachVertexFarEnoughApart(Span<const Length2D> vertices)
    {
        for (auto i = decltype(vertices.size()){1}; i < vertices.size(); ++i)
        {
            const auto delta = vertices[i-1] - vertices[i];
            
            // XXX not quite right unit-wise but this works well enough.
            if (GetLengthSquared(StripUnits(delta)) * Meter <= DefaultLinearSlop)
            {
                return false;
            }
        }
        return true;
    }
    
} // anonymous namespace

ChainShape::ChainShape(const ChainShape& other):
    Shape{Conf{}.UseVertexRadius(other.GetVertexRadius())}
{
    *this = other;
}

ChainShape& ChainShape::operator=(const ChainShape& other)
{
    if (&other != this)
    {
        Clear();
        m_count = other.m_count;
        m_vertices = other.m_vertices;
        m_normals = other.m_normals;
    }
    return *this;
}

ChainShape::~ChainShape()
{
    Clear();
}

MassData ChainShape::GetMassData() const noexcept
{
    return MassData{Mass{0}, Vec2_zero * Meter, RotInertia{0}};
}

void ChainShape::Clear()
{
    m_vertices.clear();
    m_count = 0;
}

void ChainShape::CreateLoop(Span<const Length2D> vertices)
{
    assert(vertices.begin() != nullptr);
    assert(vertices.size() >= 3);
    assert(IsEachVertexFarEnoughApart(vertices));
    assert(m_vertices.empty() && m_count == 0);
    
    const auto count = static_cast<child_count_t>(vertices.size() + 1);
    m_count = count;
    m_vertices.assign(vertices.begin(), vertices.end());
    m_vertices.push_back(m_vertices[0]);
    
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

void ChainShape::CreateChain(Span<const Length2D> vertices)
{
    assert(vertices.begin() != nullptr);
    assert(vertices.size() >= 2);
    assert(IsEachVertexFarEnoughApart(vertices));
    assert(m_vertices.empty() && m_count == 0);

    const auto count = static_cast<child_count_t>(vertices.size());
    m_count = count;
    m_vertices.assign(vertices.begin(), vertices.end());
    
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

child_count_t ChainShape::GetChildCount() const noexcept
{
    // edge count = vertex count - 1
    const auto count = GetVertexCount();
    return (count > 1)? count - 1: 0;
}

DistanceProxy ChainShape::GetChild(child_count_t index) const noexcept
{
    return DistanceProxy{GetVertexRadius(), 2, &m_vertices[index], &m_normals[index * 2]};
}

bool ChainShape::TestPoint(const Transformation& xf, const Length2D p) const noexcept
{
    NOT_USED(xf);
    NOT_USED(p);
    return false;
}
