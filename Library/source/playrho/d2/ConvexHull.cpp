/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <algorithm>
#include <cassert> // for assert
#include <iterator>

#include <playrho/d2/ConvexHull.hpp>
#include <playrho/d2/VertexSet.hpp>

namespace playrho::d2 {

ConvexHull ConvexHull::Get(const VertexSet& pointSet, NonNegative<Length> vertexRadius)
{
    auto vertices = GetConvexHullAsVector(pointSet);
    assert(!empty(vertices) && size(vertices) < std::numeric_limits<VertexCounter>::max());
    const auto count = static_cast<VertexCounter>(size(vertices));
    auto normals = std::vector<UnitVec>();
    if (count > 1) {
        // Compute normals.
        for (auto i = decltype(count){0}; i < count; ++i) {
            const auto nextIndex = GetModuloNext(i, count);
            const auto edge = vertices[nextIndex] - vertices[i];
            normals.push_back(GetUnitVector(GetFwdPerpendicular(edge)));
        }
    }
    else if (count == 1) {
        normals.emplace_back();
    }
    return ConvexHull{std::move(vertices), std::move(normals), vertexRadius};
}

ConvexHull& ConvexHull::Translate(const Length2& value)
{
    auto newPoints = VertexSet{};
    for (const auto& v : vertices) {
        newPoints.add(v + value);
    }
    *this = Get(newPoints, vertexRadius);
    return *this;
}

ConvexHull& ConvexHull::Scale(const Vec2& value)
{
    auto newPoints = VertexSet{};
    for (const auto& v : vertices) {
        newPoints.add(Length2{GetX(v) * GetX(value), GetY(v) * GetY(value)});
    }
    *this = Get(newPoints, vertexRadius);
    return *this;
}

ConvexHull& ConvexHull::Rotate(const UnitVec& value)
{
    auto newPoints = VertexSet{};
    for (const auto& v : vertices) {
        newPoints.add(::playrho::d2::Rotate(v, value));
    }
    *this = Get(newPoints, vertexRadius);
    return *this;
}

} // namespace playrho::d2
