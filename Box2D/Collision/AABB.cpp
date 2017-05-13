/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/AABB.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Shapes/Shape.hpp>
#include <Box2D/Dynamics/Fixture.hpp>
#include <Box2D/Dynamics/FixtureProxy.hpp>
#include <Box2D/Dynamics/Body.hpp>

using namespace box2d;

AABB box2d::ComputeAABB(const DistanceProxy& proxy, const Transformation xf)
{
    const auto count = proxy.GetVertexCount();
    assert(count > 0);
    auto result = AABB{Transform(proxy.GetVertex(0), xf)};
    for (auto i = decltype(count){1}; i < count; ++i)
    {
        result.Include(Transform(proxy.GetVertex(i), xf));
    }
    return result.Fatten(proxy.GetVertexRadius());
}

AABB box2d::ComputeAABB(const Shape& shape, const Transformation xf)
{
    const auto childCount = shape.GetChildCount();
    auto sum = AABB{};
    for (auto i = decltype(childCount){0}; i < childCount; ++i)
    {
        const auto dp = shape.GetChild(i);
        sum.Include(ComputeAABB(dp, xf));
    }
    return sum;
}

AABB box2d::ComputeAABB(const Body& body)
{
    auto sum = AABB{};
    const auto xf = body.GetTransformation();
    for (auto&& f: body.GetFixtures())
    {
        sum.Include(ComputeAABB(*(f->GetShape()), xf));
    }
    return sum;
}

AABB box2d::GetAABB(const Fixture& fixture, child_count_t childIndex) noexcept
{
    return fixture.GetProxy(childIndex)->aabb;
}
