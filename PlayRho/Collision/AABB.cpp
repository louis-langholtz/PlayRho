/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>
#include <PlayRho/Dynamics/Body.hpp>

/// @file
/// Definitions for the AABB class.

namespace playrho {

AABB ComputeAABB(const DistanceProxy& proxy, const Transformation& xf) noexcept
{
    assert(IsValid(xf));
    auto result = AABB{};
    const auto count = proxy.GetVertexCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        Include(result, Transform(proxy.GetVertex(i), xf));
    }
    return GetFattenedAABB(result, proxy.GetVertexRadius());
}

AABB ComputeAABB(const Shape& shape, const Transformation& xf)
{
    auto sum = AABB{};
    const auto childCount = shape.GetChildCount();
    for (auto i = decltype(childCount){0}; i < childCount; ++i)
    {
        Include(sum, ComputeAABB(shape.GetChild(i), xf));
    }
    return sum;
}

AABB ComputeAABB(const Body& body)
{
    auto sum = AABB{};
    const auto xf = body.GetTransformation();
    for (auto&& f: body.GetFixtures())
    {
        Include(sum, ComputeAABB(*(GetRef(f).GetShape()), xf));
    }
    return sum;
}

AABB GetAABB(const Fixture& fixture, ChildCounter childIndex) noexcept
{
    const auto proxy = fixture.GetProxy(childIndex);
    return proxy? proxy->aabb: AABB{};
}

} // namespace playrho
