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
#include <PlayRho/Dynamics/Joints/JointID.hpp>
#include <PlayRho/Dynamics/Joints/JointType.hpp>
#include <PlayRho/Dynamics/Joints/LimitState.hpp>

namespace playrho {
namespace d2 {

class World;
class Joint;
struct JointConf;

/// @copydoc World::CreateJoint
/// @relatedalso World
JointID CreateJoint(World& world, const Joint& def);

/// @copydoc World::Destroy(JointID)
/// @relatedalso World
void Destroy(World& world, JointID id);

/// @brief Gets the type of the joint.
/// @relatedalso World
JointType GetType(const World& world, JointID id);

const Joint& GetJoint(const World& world, JointID id);

void SetJoint(World& world, JointID id, const Joint& def);

/// @copydoc World::GetCollideConnected
/// @relatedalso World
bool GetCollideConnected(const World& world, JointID id);

/// Is the joint motor enabled?
/// @see EnableMotor(World& world, JointID joint, bool value)
/// @relatedalso World
bool IsMotorEnabled(const World& world, JointID id);

/// Enable/disable the joint motor.
/// @relatedalso World
void EnableMotor(World& world, JointID id, bool value);

/// @relatedalso World
bool IsLimitEnabled(const World& world, JointID id);

/// @relatedalso World
void EnableLimit(World& world, JointID id, bool value);

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

UnitVec GetLocalXAxisA(const World& world, JointID id);

UnitVec GetLocalYAxisA(const World& world, JointID id);

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

/// @relatedalso World
Momentum GetLinearMotorImpulse(const World& world, JointID id);

/// @brief Gets the angular motor impulse of the identified joint if it has this property.
/// @relatedalso World
AngularMomentum GetAngularMotorImpulse(const World& world, JointID id);

/// @brief Gets the computed angular rotational inertia used by the joint.
/// @relatedalso World
RotInertia GetAngularMass(const World& world, JointID id);

/// @brief Gets the frequency of the identified joint if it has this property.
/// @relatedalso World
Frequency GetFrequency(const World& world, JointID id);

/// @brief Sets the frequency of the identified joint if it has this property.
/// @relatedalso World
void SetFrequency(World& world, JointID id, Frequency value);

/// @brief Gets the angular velocity of the identified joint if it has this property.
/// @relatedalso World
AngularVelocity GetAngularVelocity(const World& world, JointID id);

/// @brief Gets the enabled/disabled state of the joint.
/// @relatedalso World
bool IsEnabled(const World& world, JointID id);

/// @brief Gets the world index of the given joint.
/// @relatedalso World
JointCounter GetWorldIndex(const World& world, JointID id) noexcept;

/// Get the anchor point on body-A in world coordinates.
/// @relatedalso World
Length2 GetAnchorA(const World& world, JointID id);

/// Get the anchor point on body-B in world coordinates.
/// @relatedalso World
Length2 GetAnchorB(const World& world, JointID id);

/// @relatedalso World
Real GetRatio(const World& world, JointID id);

/// @brief Gets the current joint translation.
/// @relatedalso World
Length GetJointTranslation(const World& world, JointID id);

/// @relatedalso World
Angle GetAngle(const World& world, JointID id);

/// @brief Gets the current motor force for the given joint, given the inverse time step.
/// @relatedalso World
inline Force GetMotorForce(const World& world, JointID id, Frequency inv_dt)
{
    return GetLinearMotorImpulse(world, id) * inv_dt;
}

/// @brief Gets the current motor torque for the given joint given the inverse time step.
/// @relatedalso World
inline Torque GetMotorTorque(const World& world, JointID id, Frequency inv_dt)
{
    return GetAngularMotorImpulse(world, id) * inv_dt;
}

/// @brief Gets the target linear offset, in frame A.
/// @relatedalso World
Length2 GetLinearOffset(const World& world, JointID id);

/// @brief Sets the target linear offset, in frame A.
/// @relatedalso World
void SetLinearOffset(World& world, JointID id, Length2 value);

/// @brief Gets the target angular offset.
/// @relatedalso World
Angle GetAngularOffset(const World& world, JointID id);

/// @brief Sets the target angular offset.
/// @relatedalso World
void SetAngularOffset(World& world, JointID id, Angle value);

/// Get the first ground anchor.
/// @relatedalso World
Length2 GetGroundAnchorA(const World& world, JointID id);

/// Get the second ground anchor.
/// @relatedalso World
Length2 GetGroundAnchorB(const World& world, JointID id);

/// @brief Get the current length of the segment attached to body-A.
/// @relatedalso World
Length GetCurrentLengthA(const World& world, JointID id);

/// @brief Get the current length of the segment attached to body-B.
/// @relatedalso World
Length GetCurrentLengthB(const World& world, JointID id);

/// @brief Gets the target point.
/// @relatedalso World
Length2 GetTarget(const World& world, JointID id);

/// @brief Sets the target point.
/// @relatedalso World
void SetTarget(World& world, JointID id, Length2 value);

/// Get the lower joint limit.
/// @relatedalso World
Angle GetAngularLowerLimit(const World& world, JointID id);

/// Get the upper joint limit.
/// @relatedalso World
Angle GetAngularUpperLimit(const World& world, JointID id);

/// Set the joint limits.
/// @relatedalso World
void SetAngularLimits(World& world, JointID id, Angle lower, Angle upper);

/// @brief Shifts the origin of the identified joint.
/// @note This only effects joints having points in world coordinates.
/// @relatedalso World
bool ShiftOrigin(World& world, JointID id, Length2 value);

/// @brief Gets the damping ratio associated with the identified joint if it has one.
/// @throws std::invalid_argument If the identified joint's type doesn't support this.
/// @relatedalso World
Real GetDampingRatio(const World& world, JointID id);

/// @brief Gets the length associated with the identified joint if it has one.
/// @throws std::invalid_argument If the identified joint's type doesn't support this.
/// @relatedalso World
Length GetLength(const World& world, JointID id);

/// @brief Gets the joint's limit state if it has one.
/// @throws std::invalid_argument If the identified joint's type doesn't support this.
/// @relatedalso World
LimitState GetLimitState(const World& world, JointID id);

/// @copydoc World::SetAwake(JointID)
/// @relatedalso World
void SetAwake(World& world, JointID id);

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_WORLDJOINT_HPP
