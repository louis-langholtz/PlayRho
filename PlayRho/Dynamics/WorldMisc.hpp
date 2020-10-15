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

#ifndef PLAYRHO_DYNAMICS_WORLDMISC_HPP
#define PLAYRHO_DYNAMICS_WORLDMISC_HPP

/// @file
/// Declarations of free functions of World for unidentified information.

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Range.hpp> // for SizedRange

#include <PlayRho/Collision/MassData.hpp>

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/BodyConf.hpp> // for GetDefaultBodyConf
#include <PlayRho/Dynamics/StepConf.hpp>
#include <PlayRho/Dynamics/StepStats.hpp>
#include <PlayRho/Dynamics/Contacts/KeyedContactID.hpp> // for KeyedContactPtr
#include <PlayRho/Dynamics/FixtureConf.hpp>
#include <PlayRho/Dynamics/WorldConf.hpp>
#include <PlayRho/Dynamics/Joints/JointID.hpp>

#include <iterator>
#include <vector> // for std::vector and std::size
#include <memory>
#include <functional>

namespace playrho {

struct FixtureProxy;

namespace d2 {

class World;
struct JointConf;
class Manifold;
class ContactImpulsesList;
class DynamicTree;

/// @brief Gets the bodies of the specified world.
/// @relatedalso World
SizedRange<std::vector<BodyID>::const_iterator> GetBodies(const World& world);

/// @brief Gets the joints of the specified world.
/// @relatedalso World
SizedRange<std::vector<JointID>::const_iterator> GetJoints(const World& world);

/// @brief Gets the contacts of the specified world.
/// @relatedalso World
SizedRange<std::vector<KeyedContactPtr>::const_iterator> GetContacts(const World& world);

/// @brief Gets the body count in the given world.
/// @return 0 or higher.
/// @relatedalso World
inline BodyCounter GetBodyCount(const World& world) noexcept
{
    using std::size;
    return static_cast<BodyCounter>(size(GetBodies(world)));
}

/// Gets the count of joints in the given world.
/// @return 0 or higher.
/// @relatedalso World
inline JointCounter GetJointCount(const World& world) noexcept
{
    using std::size;
    return static_cast<JointCounter>(size(GetJoints(world)));
}

/// @brief Gets the count of contacts in the given world.
/// @note Not all contacts are for shapes that are actually touching. Some contacts are for
///   shapes which merely have overlapping AABBs.
/// @return 0 or higher.
/// @relatedalso World
inline ContactCounter GetContactCount(const World& world) noexcept
{
    using std::size;
    return static_cast<ContactCounter>(size(GetContacts(world)));
}

/// @brief Gets the touching count for the given world.
/// @relatedalso World
ContactCounter GetTouchingCount(const World& world) noexcept;

/// @brief Steps the given world the specified amount.
/// @relatedalso World
StepStats Step(World& world, const StepConf& conf = StepConf{});

/// @brief Steps the world ahead by a given time amount.
///
/// @details Performs position and velocity updating, sleeping of non-moving bodies, updating
///   of the contacts, and notifying the contact listener of begin-contact, end-contact,
///   pre-solve, and post-solve events.
///   If the given velocity and position iterations are more than zero, this method also
///   respectively performs velocity and position resolution of the contacting bodies.
///
/// @note While body velocities are updated accordingly (per the sum of forces acting on them),
///   body positions (barring any collisions) are updated as if they had moved the entire time
///   step at those resulting velocities. In other words, a body initially at <code>p0</code>
///   going <code>v0</code> fast with a sum acceleration of <code>a</code>, after time
///   <code>t</code> and barring any collisions, will have a new velocity (<code>v1</code>) of
///   <code>v0 + (a * t)</code> and a new position (<code>p1</code>) of <code>p0 + v1 * t</code>.
///
/// @warning Varying the time step may lead to non-physical behaviors.
///
/// @post Static bodies are unmoved.
/// @post Kinetic bodies are moved based on their previous velocities.
/// @post Dynamic bodies are moved based on their previous velocities, gravity,
/// applied forces, applied impulses, masses, damping, and the restitution and friction values
/// of their fixtures when they experience collisions.
///
/// @param world World to step.
/// @param delta Time to simulate as a delta from the current state. This should not vary.
/// @param velocityIterations Number of iterations for the velocity constraint solver.
/// @param positionIterations Number of iterations for the position constraint solver.
///   The position constraint solver resolves the positions of bodies that overlap.
///
/// @relatedalso World
///
StepStats Step(World& world, Time delta,
               TimestepIters velocityIterations = 8,
               TimestepIters positionIterations = 3);

/// @copydoc World::GetTree
/// @relatedalso World
const DynamicTree& GetTree(const World& world) noexcept;

/// @copydoc World::GetFixturesForProxies
/// @relatedalso World
SizedRange<std::vector<FixtureID>::const_iterator>
GetFixturesForProxies(const World& world) noexcept;

/// @brief Gets the count of fixtures in the given world.
/// @relatedalso World
FixtureCounter GetFixtureCount(const World& world) noexcept;

/// @brief Gets the count of unique shapes in the given world.
/// @relatedalso World
std::size_t GetShapeCount(const World& world) noexcept;

/// @brief Gets the count of awake bodies in the given world.
/// @relatedalso World
BodyCounter GetAwakeCount(const World& world) noexcept;

/// @brief Awakens all of the bodies in the given world.
/// @details Calls all of the world's bodies' <code>SetAwake</code> method.
/// @return Sum total of calls to bodies' <code>SetAwake</code> method that returned true.
/// @see Body::SetAwake.
/// @relatedalso World
BodyCounter Awaken(World& world) noexcept;

/// @brief Sets the accelerations of all the world's bodies.
/// @param world World instance to set the acceleration of all contained bodies for.
/// @param fn Function or functor with a signature like:
///   <code>Acceleration (*fn)(const Body& body)</code>.
/// @relatedalso World
template <class F>
void SetAccelerations(World& world, F fn)
{
    const auto bodies = GetBodies(world);
    std::for_each(begin(bodies), end(bodies), [&](const auto &b) {
        SetAcceleration(world, b, fn(world, b));
    });
}

/// @brief Sets the accelerations of all the world's bodies to the given value.
/// @relatedalso World
void SetAccelerations(World& world, Acceleration acceleration) noexcept;

/// @brief Sets the accelerations of all the world's bodies to the given value.
/// @note This will leave the angular acceleration alone.
/// @relatedalso World
void SetAccelerations(World& world, LinearAcceleration2 acceleration) noexcept;

/// @brief Clears forces.
/// @details Manually clear the force buffer on all bodies.
/// @relatedalso World
inline void ClearForces(World& world) noexcept
{
    SetAccelerations(world, Acceleration{});
}

/// @brief Finds body in given world that's closest to the given location.
/// @relatedalso World
BodyID FindClosestBody(const World& world, Length2 location) noexcept;

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDMISC_HPP
