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
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/Contacts/Contact.hpp>

/// @file
/// Definitions for the AABB class.

namespace playrho {

AABB2D ComputeAABB(const DistanceProxy& proxy, const Transformation& xf) noexcept
{
    assert(IsValid(xf));
    auto result = AABB2D{};
    const auto count = proxy.GetVertexCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        Include(result, Transform(proxy.GetVertex(i), xf));
    }
    return GetFattenedAABB(result, proxy.GetVertexRadius());
}

AABB2D ComputeAABB(const DistanceProxy& proxy,
                   const Transformation& xfm0, const Transformation& xfm1) noexcept
{
    assert(IsValid(xfm0));
    assert(IsValid(xfm1));
    auto result = AABB2D{};
    const auto count = proxy.GetVertexCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto vertex = proxy.GetVertex(i);
        Include(result, Transform(vertex, xfm0));
        Include(result, Transform(vertex, xfm1));
    }
    return GetFattenedAABB(result, proxy.GetVertexRadius());
}

AABB2D ComputeAABB(const Shape& shape, const Transformation& xf) noexcept
{
    auto sum = AABB2D{};
    const auto childCount = shape.GetChildCount();
    for (auto i = decltype(childCount){0}; i < childCount; ++i)
    {
        Include(sum, ComputeAABB(shape.GetChild(i), xf));
    }
    return sum;
}

AABB2D ComputeAABB(const Fixture& fixture) noexcept
{
    return ComputeAABB(*fixture.GetShape(), fixture.GetBody()->GetTransformation());
}

AABB2D ComputeAABB(const Body& body)
{
    auto sum = AABB2D{};
    const auto xf = body.GetTransformation();
    for (auto&& f: body.GetFixtures())
    {
        Include(sum, ComputeAABB(*(GetRef(f).GetShape()), xf));
    }
    return sum;
}

AABB2D ComputeIntersectingAABB(const Fixture& fA, ChildCounter iA,
                             const Fixture& fB, ChildCounter iB) noexcept
{
    const auto xA = fA.GetBody()->GetTransformation();
    const auto xB = fB.GetBody()->GetTransformation();
    const auto childA = fA.GetShape()->GetChild(iA);
    const auto childB = fB.GetShape()->GetChild(iB);
    const auto aabbA = ComputeAABB(childA, xA);
    const auto aabbB = ComputeAABB(childB, xB);
    return GetIntersectingAABB(aabbA, aabbB);
}

AABB2D ComputeIntersectingAABB(const Contact& contact)
{
    return ComputeIntersectingAABB(*contact.GetFixtureA(), contact.GetChildIndexA(),
                                   *contact.GetFixtureB(), contact.GetChildIndexB());
}

} // namespace playrho
