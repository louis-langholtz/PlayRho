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

#ifndef PLAYRHO_DYNAMICS_WORLDIMPLFIXTURE_HPP
#define PLAYRHO_DYNAMICS_WORLDIMPLFIXTURE_HPP

/// @file
/// Declarations of free functions of WorldImpl for fixtures.

#include <PlayRho/Common/Units.hpp>
#include <PlayRho/Common/Transformation.hpp>
#include <PlayRho/Common/Settings.hpp> // for ChildCounter

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/Filter.hpp>

#include <PlayRho/Collision/MassData.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>

#include <vector>

namespace playrho {
namespace d2 {

class WorldImpl;
struct FixtureConf; // for CreateFixture

/// @relatedalso WorldImpl
FixtureID CreateFixture(WorldImpl& world, const FixtureConf& def, bool resetMassData = true);

/// @relatedalso WorldImpl
bool Destroy(WorldImpl& world, FixtureID id, bool resetMassData);

/// @relatedalso WorldImpl
BodyID GetBody(const WorldImpl& world, FixtureID id);

/// @relatedalso WorldImpl
Shape GetShape(const WorldImpl& world, FixtureID id);

/// @brief Is the specified fixture a sensor (non-solid)?
/// @return the true if the fixture is a sensor.
/// @relatedalso WorldImpl
bool IsSensor(const WorldImpl& world, FixtureID id);

/// @brief Gets the density of this fixture.
/// @return Non-negative density (in mass per area).
/// @relatedalso WorldImpl
AreaDensity GetDensity(const WorldImpl& world, FixtureID id);

/// @relatedalso WorldImpl
const std::vector<ContactCounter>& GetProxies(const WorldImpl& world, FixtureID id);

/// @brief Sets whether the specified fixture is a sensor or not.
/// @relatedalso WorldImpl
void SetSensor(WorldImpl& world, FixtureID id, bool value);

/// @relatedalso WorldImpl
Filter GetFilterData(const WorldImpl& world, FixtureID id);

/// @relatedalso WorldImpl
void Refilter(WorldImpl& world, FixtureID id);

/// @relatedalso WorldImpl
void SetFilterData(WorldImpl& world, FixtureID id, const Filter& value);

/// @brief Flags the contacts of the identifed fixture for filtering.
void FlagContactsForFiltering(WorldImpl& world, FixtureID id);

/// @brief Gets the count of proxies of the identified fixture.
/// @throws std::out_of_range If given an invalid fixture identifier.
/// @relatedalso World
inline ChildCounter GetProxyCount(const WorldImpl& world, FixtureID id)
{
    return static_cast<ChildCounter>(std::size(GetProxies(world, id)));
}

/// @brief Gets the specified proxy of the identified fixture.
/// @throws std::out_of_range If given an invalid fixture identifier.
/// @relatedalso World
ContactCounter GetProxy(const WorldImpl& world, FixtureID id, ChildCounter child);

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDIMPLFIXTURE_HPP
