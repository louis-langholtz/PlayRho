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

#include <PlayRho/Dynamics/WorldImplFixture.hpp>

#include <PlayRho/Dynamics/WorldImpl.hpp>
#include <PlayRho/Dynamics/WorldImplBody.hpp>
#include <PlayRho/Dynamics/Fixture.hpp> // for use of GetFixture

namespace playrho {
namespace d2 {

FixtureID CreateFixture(WorldImpl& world, BodyID id, const Shape& shape,
                        const FixtureConf& def, bool resetMassData)
{
    return world.CreateFixture(id, shape, def, resetMassData);
}

bool Destroy(WorldImpl& world, FixtureID id, bool resetMassData)
{
    return world.Destroy(id, resetMassData);
}

BodyID GetBody(const WorldImpl& world, FixtureID id)
{
    return world.GetFixture(id).GetBody();
}

Shape GetShape(const WorldImpl& world, FixtureID id)
{
    return world.GetFixture(id).GetShape();
}

bool IsSensor(const WorldImpl& world, FixtureID id)
{
    return world.GetFixture(id).IsSensor();
}

void SetSensor(WorldImpl& world, FixtureID id, bool value)
{
    world.SetSensor(id, value);
}

AreaDensity GetDensity(const WorldImpl& world, FixtureID id)
{
    return world.GetFixture(id).GetDensity();
}

const WorldImpl::Proxies& GetProxies(const WorldImpl& world, FixtureID id)
{
    return world.GetFixture(id).GetProxies();
}

Filter GetFilterData(const WorldImpl& world, FixtureID id)
{
    return world.GetFixture(id).GetFilterData();
}

void Refilter(WorldImpl& world, FixtureID id)
{
    world.Refilter(id);
}

void SetFilterData(WorldImpl& world, FixtureID id, const Filter& value)
{
    world.SetFilterData(id, value);
}

} // namespace d2
} // namespace playrho
