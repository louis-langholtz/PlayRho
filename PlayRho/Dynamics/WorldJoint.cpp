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
#include <PlayRho/Dynamics/Joints/FunctionalJointVisitor.hpp>

#include <PlayRho/Common/OptionalValue.hpp> // for Optional

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

void Accept(const World& world, JointID id, JointVisitor& visitor)
{
    world.Accept(id, visitor);
}

void Accept(World& world, JointID id, JointVisitor& visitor)
{
    world.Accept(id, visitor);
}

Real GetRatio(const World& world, JointID id)
{
    Optional<Real> result;
    FunctionalJointVisitor visitor;
    visitor.get<const GearJoint&>() = [&result](const GearJoint& j) {
        result = j.GetRatio();
    };
    visitor.get<const PulleyJoint&>() = [&result](const PulleyJoint& j) {
        result = j.GetRatio();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetRatio not supported by joint type!");
    }
    return *result;
}

Length GetJointTranslation(const World& world, JointID id)
{
    const auto pA = GetWorldPoint(world, GetBodyA(world, id), GetLocalAnchorA(world, id));
    const auto pB = GetWorldPoint(world, GetBodyB(world, id), GetLocalAnchorB(world, id));
    const auto uv = GetWorldVector(world, GetBodyA(world, id), GetLocalAxisA(world, id));
    return Dot(pB - pA, uv);
}

Angle GetAngle(const World& world, JointID id)
{
    return GetAngle(world, GetBodyB(world, id)) - GetAngle(world, GetBodyA(world, id)) - GetReferenceAngle(world, id);
}

bool IsLimitEnabled(const World& world, JointID id)
{
    Optional<bool> result;
    FunctionalJointVisitor visitor;
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.IsLimitEnabled();
    };
    visitor.get<const PrismaticJoint&>() = [&result](const PrismaticJoint& j) {
        result = j.IsLimitEnabled();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("IsLimitEnabled not supported by joint type!");
    }
    return *result;
}

void EnableLimit(World& world, JointID id, bool value)
{
    // TODO
}

Momentum GetLinearMotorImpulse(const World& world, JointID id)
{
    Optional<Momentum> result;
    FunctionalJointVisitor visitor;
    visitor.get<const PrismaticJoint&>() = [&result](const PrismaticJoint& j) {
        result = j.GetLinearMotorImpulse();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetLinearMotorImpulse not supported by joint type!");
    }
    return *result;
}

Length2 GetLinearOffset(const World& world, JointID id)
{
    Optional<Length2> result;
    FunctionalJointVisitor visitor;
    visitor.get<const MotorJoint&>() = [&result](const MotorJoint& j) {
        result = j.GetLinearOffset();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetLinearOffset not supported by joint type!");
    }
    return *result;
}

void SetLinearOffset(World& world, JointID id, Length2 value)
{
    // TODO
}

Angle GetAngularOffset(const World& world, JointID id)
{
    Optional<Angle> result;
    FunctionalJointVisitor visitor;
    visitor.get<const MotorJoint&>() = [&result](const MotorJoint& j) {
        result = j.GetAngularOffset();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetAngularOffset not supported by joint type!");
    }
    return *result;
}

void SetAngularOffset(World& world, JointID id, Angle value)
{
    // TODO
}

Length2 GetGroundAnchorA(const World& world,  JointID id)
{
    Optional<Length2> result;
    FunctionalJointVisitor visitor;
    visitor.get<const PulleyJoint&>() = [&result](const PulleyJoint& j) {
        result = j.GetGroundAnchorA();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetGroundAnchorA not supported by joint type!");
    }
    return *result;
}

Length2 GetGroundAnchorB(const World& world,  JointID id)
{
    Optional<Length2> result;
    FunctionalJointVisitor visitor;
    visitor.get<const PulleyJoint&>() = [&result](const PulleyJoint& j) {
        result = j.GetGroundAnchorB();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetGroundAnchorB not supported by joint type!");
    }
    return *result;
}

Length GetCurrentLengthA(const World& world, JointID id)
{
    return GetMagnitude(GetWorldPoint(world, GetBodyA(world, id),
                                      GetLocalAnchorA(world, id)) - GetGroundAnchorA(world, id));
}

Length GetCurrentLengthB(const World& world, JointID id)
{
    return GetMagnitude(GetWorldPoint(world, GetBodyB(world, id),
                                      GetLocalAnchorB(world, id)) - GetGroundAnchorB(world, id));
}

Length2 GetTarget(const World& world, JointID id)
{
    return world.GetTarget(id);
}

void SetTarget(World& world, JointID id, Length2 value)
{
    world.SetTarget(id, value);
}

Angle GetAngularLowerLimit(const World& world, JointID id)
{
    return world.GetAngularLowerLimit(id);
}

Angle GetAngularUpperLimit(const World& world, JointID id)
{
    return world.GetAngularUpperLimit(id);
}

void SetAngularLimits(World& world, JointID id, Angle lower, Angle upper)
{
    world.SetAngularLimits(id, lower, upper);
}

} // namespace d2
} // namespace playrho
