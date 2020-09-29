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

#include <PlayRho/Dynamics/WorldJoint.hpp>

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/WorldBody.hpp>

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/JointVisitor.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/Joints/TargetJoint.hpp>
#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Dynamics/Joints/WheelJoint.hpp>
#include <PlayRho/Dynamics/Joints/WeldJoint.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJoint.hpp>
#include <PlayRho/Dynamics/Joints/RopeJoint.hpp>
#include <PlayRho/Dynamics/Joints/MotorJoint.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

namespace playrho {
namespace d2 {

JointType GetType(const World& world, JointID id)
{
    return world.GetType(id);
}

bool GetCollideConnected(const World& world, JointID id)
{
    return world.GetCollideConnected(id);
}

bool IsMotorEnabled(const World& world, JointID id)
{
    return world.IsMotorEnabled(id);
}

void EnableMotor(World& world, JointID id, bool value)
{
    world.EnableMotor(id, value);
}

void* GetUserData(const World& world, JointID id)
{
    return world.GetUserData(id);
}

BodyID GetBodyA(const World& world, JointID id)
{
    return world.GetBodyA(id);
}

BodyID GetBodyB(const World& world, JointID id)
{
    return world.GetBodyB(id);
}

Length2 GetLocalAnchorA(const World& world, JointID id)
{
    return world.GetLocalAnchorA(id);
}

Length2 GetLocalAnchorB(const World& world, JointID id)
{
    return world.GetLocalAnchorB(id);
}

Momentum2 GetLinearReaction(const World& world, JointID id)
{
    return world.GetLinearReaction(id);
}

AngularMomentum GetAngularReaction(const World& world, JointID id)
{
    return world.GetAngularReaction(id);
}

Angle GetReferenceAngle(const World& world, JointID id)
{
    return world.GetReferenceAngle(id);
}

UnitVec GetLocalAxisA(const World& world, JointID id)
{
    return world.GetLocalAxisA(id);
}

AngularVelocity GetMotorSpeed(const World& world, JointID id)
{
    return world.GetMotorSpeed(id);
}

void SetMotorSpeed(World& world, JointID id, AngularVelocity value)
{
    world.SetMotorSpeed(id, value);
}

Torque GetMaxMotorTorque(const World& world, JointID id)
{
    return world.GetMaxMotorTorque(id);
}

void SetMaxMotorTorque(World& world, JointID id, Torque value)
{
    world.SetMaxMotorTorque(id, value);
}

AngularMomentum GetAngularMotorImpulse(const World& world, JointID id)
{
    return world.GetAngularMotorImpulse(id);
}

RotInertia GetAngularMass(const World& world, JointID id)
{
    return world.GetAngularMass(id);
}

Frequency GetFrequency(const World& world, JointID id)
{
    return world.GetFrequency(id);
}

void SetFrequency(World& world, JointID id, Frequency value)
{
    world.SetFrequency(id, value);
}

AngularVelocity GetAngularVelocity(const World& world, JointID id)
{
    return GetVelocity(world, GetBodyB(world, id)).angular
         - GetVelocity(world, GetBodyA(world, id)).angular;
}

bool IsEnabled(const World& world, JointID id)
{
    const auto bA = GetBodyA(world, id);
    const auto bB = GetBodyB(world, id);
    return (bA == InvalidBodyID || IsEnabled(world, bA))
        && (bB == InvalidBodyID || IsEnabled(world, bB));
}

JointCounter GetWorldIndex(const World& world, JointID id) noexcept
{
    const auto elems = world.GetJoints();
    const auto it = std::find(cbegin(elems), cend(elems), id);
    if (it != cend(elems))
    {
        return static_cast<JointCounter>(std::distance(cbegin(elems), it));
    }
    return JointCounter(-1);
}

Length2 GetAnchorA(const World& world, JointID id)
{
    return GetWorldPoint(world, GetBodyA(world, id), GetLocalAnchorA(world, id));
}

Length2 GetAnchorB(const World& world, JointID id)
{
    return GetWorldPoint(world, GetBodyB(world, id), GetLocalAnchorB(world, id));
}

} // namespace d2
} // namespace playrho
