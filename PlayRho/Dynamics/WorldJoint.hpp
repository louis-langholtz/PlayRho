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

#ifndef PLAYRHO_DYNAMICS_WORLDJOINT_HPP
#define PLAYRHO_DYNAMICS_WORLDJOINT_HPP

/// @file
/// Declarations of free functions of World for joints identified by <code>JointID</code>.

#include <PlayRho/Common/Math.hpp>

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/FixtureID.hpp>
#include <PlayRho/Dynamics/Joints/JointID.hpp>
#include <PlayRho/Dynamics/Joints/JointType.hpp>

namespace playrho {
namespace d2 {

class World;
struct JointConf;

/// @brief Gets the type of the joint.
/// @relatedalso World
JointType GetType(const World& world, JointID id);

/// @copydoc World::GetCollideConnected
/// @relatedalso World
bool GetCollideConnected(const World& world, JointID id);

/// @copydoc World::IsMotorEnabled()
/// @relatedalso World
bool IsMotorEnabled(const World& world, JointID id);

/// @copydoc World::EnableMotor()
/// @relatedalso World
void EnableMotor(World& world, JointID id, bool value);

/// @copydoc World::GetUserData(JointID)
/// @relatedalso World
void* GetUserData(const World& world, JointID id);

BodyID GetBodyA(const World& world, JointID id);

BodyID GetBodyB(const World& world, JointID id);

/// Get the anchor point on body-A in local coordinates.
Length2 GetLocalAnchorA(const World& world, JointID id);

/// Get the anchor point on body-B in local coordinates.
Length2 GetLocalAnchorB(const World& world, JointID id);

/// @copydoc World::GetLinearReaction
Momentum2 GetLinearReaction(const World& world, JointID id);

/// @copydoc World::GetAngularReaction
AngularMomentum GetAngularReaction(const World& world, JointID id);

Angle GetReferenceAngle(const World& world, JointID id);

UnitVec GetLocalAxisA(const World& world, JointID id);

/// @copydoc World::GetMotorSpeed
/// @see SetMotorSpeed(World& world, JointID id, AngularVelocity value)
AngularVelocity GetMotorSpeed(const World& world, JointID id);

/// @copydoc World::SetMotorSpeed
/// @see GetMotorSpeed(const World& world, JointID id)
void SetMotorSpeed(World& world, JointID id, AngularVelocity value);

/// @brief Gets the max motor torque.
Torque GetMaxMotorTorque(const World& world, JointID id);

/// Set the maximum motor torque.
void SetMaxMotorTorque(World& world, JointID id, Torque value);

/// @copydoc World::GetAngularMotorImpulse
/// @relatedalso World
AngularMomentum GetAngularMotorImpulse(const World& world, JointID id);

/// @copydoc World::GetAngularMass
/// @relatedalso World
RotInertia GetAngularMass(const World& world, JointID id);

/// @copydoc World::GetFrequency
/// @relatedalso World
Frequency GetFrequency(const World& world, JointID id);

/// @copydoc World::GetFrequency
/// @relatedalso World
void SetFrequency(World& world, JointID id, Frequency value);

/// @brief Gets the angular velocity of the identified joint if it has this property.
/// @relatedalso World
AngularVelocity GetAngularVelocity(const World& world, JointID id);

/// @brief Gets the enabled/disabled state of the joint.
bool IsEnabled(const World& world, JointID id);

/// @brief Gets the world index of the given joint.
/// @relatedalso World
JointCounter GetWorldIndex(const World& world, JointID id) noexcept;

/// Get the anchor point on body-A in world coordinates.
Length2 GetAnchorA(const World& world, JointID id);

/// Get the anchor point on body-B in world coordinates.
Length2 GetAnchorB(const World& world, JointID id);

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDJOINT_HPP
