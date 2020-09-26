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

#ifndef PLAYRHO_DYNAMICS_WORLDIMPLJOINT_HPP
#define PLAYRHO_DYNAMICS_WORLDIMPLJOINT_HPP

/// @file
/// Declarations of free functions of WorldImpl for joints.

#include <PlayRho/Common/Units.hpp>

#include <PlayRho/Dynamics/BodyID.hpp>
#include <PlayRho/Dynamics/Joints/JointID.hpp>
#include <PlayRho/Dynamics/Joints/JointType.hpp>

namespace playrho {
namespace d2 {

class WorldImpl;

/// @brief Destroys the identified joint.
void Destroy(WorldImpl& world, JointID id);

/// @brief Gets the type of the joint.
JointType GetType(const WorldImpl& world, JointID id);

/// @brief Gets the linear reaction on body-B at the joint anchor.
Momentum2 GetLinearReaction(const WorldImpl& world, JointID id);

/// @brief Get the angular reaction on body-B for the identified joint.
AngularMomentum GetAngularReaction(const WorldImpl& world, JointID id);

/// Is the joint motor enabled?
/// @see EnableMotor(WorldImpl& world, JointID joint, bool value)
bool IsMotorEnabled(const WorldImpl& world, JointID id);

/// Enable/disable the joint motor.
void EnableMotor(WorldImpl& world, JointID joint, bool value);

/// @brief Wakes up the joined bodies.
/// @relatedalso WorldImpl
void SetAwake(WorldImpl& world, JointID id);

/// @brief Gets collide connected for the specified joint.
/// @note Modifying the collide connect flag won't work correctly because
///   the flag is only checked when fixture AABBs begin to overlap.
bool GetCollideConnected(const WorldImpl& world, JointID id);

/// @brief Gets the user data associated with the identified joint.
/// @relatedalso WorldImpl
void* GetUserData(const WorldImpl& world, JointID id);

BodyID GetBodyA(const WorldImpl& world, JointID id);

BodyID GetBodyB(const WorldImpl& world, JointID id);

Length2 GetLocalAnchorA(const WorldImpl& world, JointID id);

Length2 GetLocalAnchorB(const WorldImpl& world, JointID id);

Angle GetReferenceAngle(const WorldImpl& world, JointID id);

UnitVec GetLocalAxisA(const WorldImpl& world, JointID id);

/// @brief Gets the angular motor speed for joints which support this.
/// @see SetMotorSpeed(JointID id, AngularVelocity value)
AngularVelocity GetMotorSpeed(const WorldImpl& world, JointID id);

/// @brief Sets the angular motor speed for joints which support this.
/// @see GetMotorSpeed(const WorldImpl& world, JointID id) const
void SetMotorSpeed(WorldImpl& world, JointID id, AngularVelocity value);

/// @brief Gets the max motor torque.
Torque GetMaxMotorTorque(const WorldImpl& world, JointID id);

/// Sets the maximum motor torque.
void SetMaxMotorTorque(WorldImpl& world, JointID id, Torque value);

/// @brief Gets the angular motor impulse of the identified joint.
AngularMomentum GetAngularMotorImpulse(const WorldImpl& world, JointID id);

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDIMPLJOINT_HPP
