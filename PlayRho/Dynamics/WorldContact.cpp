/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/WorldContact.hpp>

#include <PlayRho/Dynamics/World.hpp>

namespace playrho {
namespace d2 {

bool IsTouching(const World& world, ContactID id)
{
    return world.IsTouching(id);
}

bool IsAwake(const World& world, ContactID id)
{
    return world.IsAwake(id);
}

void SetAwake(World& world, ContactID id)
{
    world.SetAwake(id);
}

FixtureID GetFixtureA(const World& world, ContactID id)
{
    return world.GetFixtureA(id);
}

FixtureID GetFixtureB(const World& world, ContactID id)
{
    return world.GetFixtureB(id);
}

BodyID GetBodyA(const World& world, ContactID id)
{
    return world.GetBodyA(id);
}

BodyID GetBodyB(const World& world, ContactID id)
{
    return world.GetBodyB(id);
}

TimestepIters GetToiCount(const World& world, ContactID id)
{
    return world.GetToiCount(id);
}

bool NeedsFiltering(const World& world, ContactID id)
{
    return world.NeedsFiltering(id);
}

bool NeedsUpdating(const World& world, ContactID id)
{
    return world.NeedsUpdating(id);
}

bool HasValidToi(const World& world, ContactID id)
{
    return world.HasValidToi(id);
}

Real GetDefaultFriction(const World& world, ContactID id)
{
    return world.GetDefaultFriction(id);
}

Real GetDefaultRestitution(const World& world, ContactID id)
{
    return world.GetDefaultRestitution(id);
}

Real GetFriction(const World& world, ContactID id)
{
    return world.GetFriction(id);
}

Real GetRestitution(const World& world, ContactID id)
{
    return world.GetRestitution(id);
}

void SetFriction(World& world, ContactID id, Real friction)
{
    world.SetFriction(id, friction);
}

void SetRestitution(World& world, ContactID id, Real restitution)
{
    world.SetRestitution(id, restitution);
}

const Manifold& GetManifold(const World& world, ContactID id)
{
    return world.GetManifold(id);
}

} // namespace d2
} // namespace playrho
