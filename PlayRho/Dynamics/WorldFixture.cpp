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

#include <PlayRho/Dynamics/WorldFixture.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>
#include <PlayRho/Dynamics/WorldShape.hpp>
#include <PlayRho/Dynamics/Body.hpp> // for GetBody
#include <PlayRho/Dynamics/Contacts/Contact.hpp> // for MixFriction

namespace playrho {
namespace d2 {

using playrho::size;

bool TestPoint(const World& world, FixtureID id, Length2 p)
{
    return TestPoint(GetShape(world, id), InverseTransform(p, GetTransformation(world, id)));
}

NonNegative<AreaDensity> GetDensity(const World& world, const FixtureConf& conf)
{
    return GetDensity(world.GetShape(GetShape(conf)));
}

Real GetDefaultFriction(const World& world,
                        const FixtureConf& fixtureA, const FixtureConf& fixtureB)
{
    return MixFriction(GetFriction(world.GetShape(GetShape(fixtureA))),
                       GetFriction(world.GetShape(GetShape(fixtureB))));
}

Real GetDefaultRestitution(const World& world,
                           const FixtureConf& fixtureA, const FixtureConf& fixtureB)
{
    return MixRestitution(GetRestitution(world.GetShape(GetShape(fixtureA))),
                          GetRestitution(world.GetShape(GetShape(fixtureB))));
}

} // namespace d2
} // namespace playrho
