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

#include <PlayRho/Collision/Shapes/MultiShapeConf.hpp>
#include <PlayRho/Common/VertexSet.hpp>
#include <algorithm>

namespace playrho {
namespace d2 {

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @return Mass data for this shape.
MassData GetMassData(const MultiShapeConf& arg) noexcept
{
    auto mass = 0_kg;
    const auto origin = Length2{};
    auto weightedCenter = origin * Kilogram;
    auto I = RotInertia(0);
    const auto vertexRadius = arg.vertexRadius;
    const auto density = arg.density;

    std::for_each(std::begin(arg.children), std::end(arg.children),
                  [&](const ConvexHull& ch) {
        const auto dp = ch.GetDistanceProxy(vertexRadius);
        const auto md = playrho::d2::GetMassData(vertexRadius, density,
            Span<const Length2>(dp.GetVertices().begin(), dp.GetVertexCount()));
        mass += Mass{md.mass};
        weightedCenter += md.center * Mass{md.mass};
        I += RotInertia{md.I};
    });

    const auto center = (mass > 0_kg)? weightedCenter / mass: origin;
    return MassData{center, mass, I};
}

ConvexHull ConvexHull::Get(const VertexSet& pointSet)
{
    auto vertices = GetConvexHullAsVector(pointSet);
    assert(!vertices.empty() && vertices.size() < std::numeric_limits<VertexCounter>::max());
    
    const auto count = static_cast<VertexCounter>(vertices.size());
    
    auto normals = std::vector<UnitVec>();
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
        normals.push_back(UnitVec{});
    }
    
    return ConvexHull{vertices, normals};
}

MultiShapeConf& MultiShapeConf::AddConvexHull(const VertexSet& pointSet) noexcept
{
    children.emplace_back(ConvexHull::Get(pointSet));
    return *this;
}

} // namespace d2
} // namespace playrho
