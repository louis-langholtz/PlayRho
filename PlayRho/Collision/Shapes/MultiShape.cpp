/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

using namespace box2d;

/// Computes the mass properties of this shape using its dimensions and density.
/// The inertia tensor is computed about the local origin.
/// @return Mass data for this shape.
MassData MultiShape::GetMassData() const noexcept
{
    // TODO
    return MassData{};
}

void MultiShape::AddConvexHull(const VertexSet& point_set) noexcept
{
#ifndef NDEBUG
    const auto point_set_size = point_set.size();
    assert(point_set_size > 0 && point_set_size < std::numeric_limits<vertex_count_t>::max());
#endif
    
    auto vertices = GetConvexHullAsVector(point_set);
    assert(vertices.size() > 0 && vertices.size() < std::numeric_limits<vertex_count_t>::max());
    
    const auto count = static_cast<vertex_count_t>(vertices.size());
    
    auto normals = std::vector<UnitVec2>();
    if (count > 1)
    {
        // Compute normals.
        for (auto i = decltype(count){0}; i < count; ++i)
        {
            const auto nextIndex = GetModuloNext(i, count);
            const auto edge = vertices[nextIndex] - vertices[i];
            normals.emplace_back(GetUnitVector(GetFwdPerpendicular(edge)));
        }
    }
    else if (count == 1)
    {
        normals.emplace_back(UnitVec2{});
    }
    
    // TODO: Compute the polygon centroid.
    
    m_children.push_back(ConvexHull{vertices, normals});
}
