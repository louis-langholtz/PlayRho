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

#include <PlayRho/Dynamics/WorldMisc.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>

#include <PlayRho/Dynamics/BodyConf.hpp>
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>
#include <PlayRho/Dynamics/MovementConf.hpp>

#include <algorithm> // for std::for_each

using std::for_each;

namespace playrho {
namespace d2 {

using playrho::size;

SizedRange<std::vector<BodyID>::const_iterator> GetBodies(const World& world)
{
    return world.GetBodies();
}

SizedRange<std::vector<JointID>::const_iterator> GetJoints(const World& world)
{
    return world.GetJoints();
}

SizedRange<std::vector<KeyedContactPtr>::const_iterator> GetContacts(const World& world)
{
    return world.GetContacts();
}

StepStats Step(World& world, const StepConf& conf)
{
    return world.Step(conf);
}

StepStats Step(World& world, Time delta, TimestepIters velocityIterations,
               TimestepIters positionIterations)
{
    StepConf conf;
    conf.SetTime(delta);
    conf.regVelocityIterations = velocityIterations;
    conf.regPositionIterations = positionIterations;
    conf.toiVelocityIterations = velocityIterations;
    if (positionIterations == 0)
    {
        conf.toiPositionIterations = 0;
    }
    conf.dtRatio = delta * world.GetInvDeltaTime();
    return world.Step(conf);
}

const DynamicTree& GetTree(const World& world) noexcept
{
    return world.GetTree();
}

SizedRange<std::vector<FixtureID>::const_iterator>
GetFixturesForProxies(const World& world) noexcept
{
    return world.GetFixturesForProxies();
}

ContactCounter GetTouchingCount(const World& world) noexcept
{
    const auto contacts = world.GetContacts();
    return static_cast<ContactCounter>(count_if(cbegin(contacts), cend(contacts),
                                                [&](const auto &c) {
        return world.IsTouching(std::get<ContactID>(c));
    }));
}

FixtureCounter GetFixtureCount(const World& world) noexcept
{
    auto sum = FixtureCounter{0};
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world,&sum](const auto &b) {
        sum += GetFixtureCount(world, b);
    });
    return sum;
}

size_t GetShapeCount(const World& world) noexcept
{
    return world.GetShapeCount();
}

BodyCounter GetAwakeCount(const World& world) noexcept
{
    const auto bodies = world.GetBodies();
    return static_cast<BodyCounter>(count_if(cbegin(bodies), cend(bodies),
                                             [&](const auto &b) {
                                                 return IsAwake(world, b); }));
}
    
BodyCounter Awaken(World& world) noexcept
{
    // Can't use count_if since body gets modified.
    auto awoken = BodyCounter{0};
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world,&awoken](const auto &b) {
        if (::playrho::d2::Awaken(world, b))
        {
            ++awoken;
        }
    });
    return awoken;
}

void SetAccelerations(World& world, Acceleration acceleration) noexcept
{
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world, acceleration](const auto &b) {
        SetAcceleration(world, b, acceleration);
    });
}

void SetAccelerations(World& world, LinearAcceleration2 acceleration) noexcept
{
    const auto bodies = world.GetBodies();
    for_each(begin(bodies), end(bodies), [&world, acceleration](const auto &b) {
        SetAcceleration(world, b, acceleration);
    });
}

BodyID FindClosestBody(const World& world, Length2 location) noexcept
{
    const auto bodies = world.GetBodies();
    auto found = InvalidBodyID;
    auto minLengthSquared = std::numeric_limits<Area>::infinity();
    for (const auto& body: bodies)
    {
        const auto bodyLoc = GetLocation(world, body);
        const auto lengthSquared = GetMagnitudeSquared(bodyLoc - location);
        if (minLengthSquared > lengthSquared)
        {
            minLengthSquared = lengthSquared;
            found = body;
        }
    }
    return found;
}

} // namespace d2
} // namespace playrho
