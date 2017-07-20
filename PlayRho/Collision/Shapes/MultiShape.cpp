/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Collision/Shapes/MultiShape.hpp>
#include <PlayRho/Common/VertexSet.hpp>
#include <algorithm>

using namespace playrho;

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @return Mass data for this shape.
MassData MultiShape::GetMassData() const noexcept
{
    auto mass = Mass(Real(0) * Kilogram);
    const auto origin = Length2D(Real(0) * Meter, Real(0) * Meter);
    auto weightedCenter = origin * Kilogram;
    auto I = RotInertia(0);

    std::for_each(std::begin(m_children), std::end(m_children), [&](const ConvexHull& ch) {
        const auto md = ::GetMassData(GetVertexRadius(), GetDensity(),
                                      Span<const Length2D>(ch.vertices.data(), ch.vertices.size()));
        mass += Mass{md.mass};
        weightedCenter += md.center * Mass{md.mass};
        I += RotInertia{md.I};
    });

    const auto center = (mass > Mass{0})? weightedCenter / mass: origin;
    return MassData{mass, center, I};
}

void MultiShape::AddConvexHull(const VertexSet& pointSet) noexcept
{
#ifndef NDEBUG
    const auto point_set_size = pointSet.size();
    assert(point_set_size > 0 && point_set_size < std::numeric_limits<vertex_count_t>::max());
#endif
    
    auto vertices = GetConvexHullAsVector(pointSet);
    assert(vertices.size() > 0 && vertices.size() < std::numeric_limits<vertex_count_t>::max());
    
    const auto count = static_cast<VertexCounter>(vertices.size());
    
    auto normals = std::vector<UnitVec2>();
    if (count > 1)
    {
        // Compute normals.
        for (auto i = decltype(count){0}; i < count; ++i)
        {
            const auto nextIndex = GetModuloNext(i, count);
            const auto edge = vertices[nextIndex] - vertices[i];
            normals.push_back(GetUnitVector(GetFwdPerpendicular(edge)));
        }
    }
    else if (count == 1)
    {
        normals.push_back(UnitVec2{});
    }
    
    m_children.push_back(ConvexHull{vertices, normals});
}
