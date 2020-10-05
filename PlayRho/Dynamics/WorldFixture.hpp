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

#ifndef PLAYRHO_DYNAMICS_WORLDFIXTURE_HPP
#define PLAYRHO_DYNAMICS_WORLDFIXTURE_HPP

/// @file
/// Declarations of free functions of World for fixtures identified by <code>FixtureID</code>.

#include <PlayRho/Common/Math.hpp>

#include <PlayRho/Collision/MassData.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/FixtureConf.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>

#include <iterator>
#include <vector>

namespace playrho {

struct Filter;

namespace d2 {

class World;

/// @relatedalso World
FixtureID CreateFixture(World& world, BodyID id, const Shape& shape,
                        const FixtureConf& def = GetDefaultFixtureConf(),
                        bool resetMassData = true);

/// @copydoc World::Destroy(FixtureID id, bool resetMassData)
/// @relatedalso World
bool Destroy(World& world, FixtureID id, bool resetMassData = true);

/// @copydoc World::GetFilterData
/// @relatedalso World
Filter GetFilterData(const World& world, FixtureID id);

/// @copydoc World::SetFilterData
/// @relatedalso World
void SetFilterData(World& world, FixtureID id, const Filter& filter);

/// @copydoc World::Refilter
/// @relatedalso World
void Refilter(World& world, FixtureID id);

/// @relatedalso World
BodyID GetBody(const World& world, FixtureID id);

/// @copydoc World::GetUserData(FixtureID)
/// @relatedalso World
void* GetUserData(const World& world, FixtureID id);

void SetUserData(World& world, FixtureID id, void* value);

/// @brief Gets the transformation associated with the given fixture.
/// @warning Behavior is undefined if the fixture doesn't have an associated body - i.e.
///   behavior is undefined if the fixture has <code>nullptr</code> as its associated body.
/// @relatedalso World
Transformation GetTransformation(const World& world, FixtureID id);

Shape GetShape(const World& world, FixtureID id);

/// @brief Gets the coefficient of friction of the specified fixture.
/// @return Value of 0 or higher.
inline Real GetFriction(const World& world, FixtureID id)
{
    return GetFriction(GetShape(world, id));
}

/// @brief Gets the coefficient of restitution of the specified fixture.
inline Real GetRestitution(const World& world, FixtureID id)
{
    return GetRestitution(GetShape(world, id));
}

/// @brief Sets whether the fixture is a sensor or not.
/// @see IsSensor.
void SetSensor(World& world, FixtureID id, bool value);

/// @brief Is the specified fixture a sensor (non-solid)?
/// @return the true if the fixture is a sensor.
/// @see SetSensor.
bool IsSensor(const World& world, FixtureID id);

/// @brief Gets the density of this fixture.
/// @return Non-negative density (in mass per area).
AreaDensity GetDensity(const World& world, FixtureID id);

const std::vector<FixtureProxy>& GetProxies(const World& world, FixtureID id);

/// @relatedalso World
inline ChildCounter GetProxyCount(const World& world, FixtureID id)
{
    return static_cast<ChildCounter>(std::size(GetProxies(world, id)));
}

/// @relatedalso World
const FixtureProxy& GetProxy(const World& world, FixtureID id, ChildCounter child);

inline MassData GetMassData(const World& world, FixtureID id)
{
    return GetMassData(GetShape(world, id));
}

/// @brief Tests a point for containment in a fixture.
/// @param id Fixture to use for test.
/// @param p Point in world coordinates.
/// @relatedalso World
/// @ingroup TestPointGroup
bool TestPoint(const World& world, FixtureID id, Length2 p);

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDFIXTURE_HPP
