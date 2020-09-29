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

#ifndef PLAYRHO_DYNAMICS_WORLDCONTACT_HPP
#define PLAYRHO_DYNAMICS_WORLDCONTACT_HPP

/// @file
/// Declarations of free functions of World for contacts identified by <code>ContactID</code>.

#include <PlayRho/Common/Settings.hpp>

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/Contacts/ContactID.hpp>

namespace playrho {
namespace d2 {

class World;
class Manifold;

/// @copydoc World::IsTouching(ContactID)
bool IsTouching(const World& world, ContactID id);

/// @copydoc World::IsAwake(ContactID)
bool IsAwake(const World& world, ContactID id);

/// @brief Sets awake the bodies of the fixtures of the given contact.
/// @relatedalso World
void SetAwake(World& world, ContactID id);

/// @copydoc World::GetFixtureA
/// @relatedalso World
FixtureID GetFixtureA(const World& world, ContactID id);

/// @copydoc World::GetFixtureB
/// @relatedalso World
FixtureID GetFixtureB(const World& world, ContactID id);

BodyID GetBodyA(const World& world, ContactID id);

BodyID GetBodyB(const World& world, ContactID id);

TimestepIters GetToiCount(const World& world, ContactID id);

/// @copydoc World::NeedsFiltering
/// @relatedalso World
bool NeedsFiltering(const World& world, ContactID id);

/// @copydoc World::NeedsUpdating
/// @relatedalso World
bool NeedsUpdating(const World& world, ContactID id);

/// @copydoc World::HasValidToi
/// @relatedalso World
bool HasValidToi(const World& world, ContactID id);

Real GetDefaultFriction(const World& world, ContactID id);

Real GetDefaultRestitution(const World& world, ContactID id);

/// @copydoc World::GetFriction(ContactID id)
/// @see SetFriction(World& world, ContactID id, Real friction)
Real GetFriction(const World& world, ContactID id);

/// @copydoc GetRestitution(ContactID id)
/// @see SetRestitution(World& world, ContactID id, Real restitution)
Real GetRestitution(const World& world, ContactID id);

/// @brief Sets the friction value for the specified contact.
/// @details Overrides the default friction mixture.
/// @note You can call this in "pre-solve" listeners.
/// @note This value persists until set or reset.
/// @warning Behavior is undefined if given a negative friction value.
/// @param friction Co-efficient of friction value of zero or greater.
/// @relatedalso World
void SetFriction(World& world, ContactID id, Real friction);

/// @brief Sets the restitution value for the specified contact.
/// @details This override the default restitution mixture.
/// @note You can call this in "pre-solve" listeners.
/// @note The value persists until you set or reset.
/// @relatedalso World
void SetRestitution(World& world, ContactID id, Real restitution);

const Manifold& GetManifold(const World& world, ContactID id);

/// Resets the friction mixture to the default value.
/// @relatedalso World
inline void ResetFriction(World& world, ContactID id)
{
    SetFriction(world, id, GetDefaultFriction(world, id));
}

/// Resets the restitution to the default value.
/// @relatedalso World
inline void ResetRestitution(World& world, ContactID id)
{
    SetRestitution(world, id, GetDefaultRestitution(world, id));
}

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDCONTACT_HPP
