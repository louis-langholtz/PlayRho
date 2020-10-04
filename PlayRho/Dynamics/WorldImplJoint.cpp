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

#include <PlayRho/Dynamics/WorldImplJoint.hpp>

#include <PlayRho/Dynamics/WorldImpl.hpp>
#include <PlayRho/Dynamics/Body.hpp> // for use of GetBody(BodyID)

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/JointVisitor.hpp>
#include <PlayRho/Dynamics/Joints/FunctionalJointVisitor.hpp>
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

#include <PlayRho/Common/OptionalValue.hpp> // for Optional

namespace playrho {
namespace d2 {

void Destroy(WorldImpl& world, JointID id)
{
    world.Destroy(id);
}

JointType GetType(const WorldImpl& world, JointID id)
{
    return ::playrho::d2::GetType(world.GetJoint(id));
}

Momentum2 GetLinearReaction(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetLinearReaction();
}

AngularMomentum GetAngularReaction(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetAngularReaction();
}

void SetAwake(WorldImpl& world, JointID id)
{
    const auto& joint = world.GetJoint(id);
    const auto bA = joint.GetBodyA();
    const auto bB = joint.GetBodyB();
    if (bA != InvalidBodyID)
    {
        world.GetBody(bA).SetAwake();
    }
    if (bB != InvalidBodyID)
    {
        world.GetBody(bB).SetAwake();
    }
}

bool GetCollideConnected(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetCollideConnected();
}

void* GetUserData(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetUserData();
}

void SetUserData(WorldImpl& world, JointID id, void* value)
{
    world.GetJoint(id).SetUserData(value);
}

BodyID GetBodyA(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetBodyA();
}

BodyID GetBodyB(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetBodyB();
}

Length2 GetLocalAnchorA(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetLocalAnchorA();
}

Length2 GetLocalAnchorB(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id).GetLocalAnchorB();
}

Angle GetReferenceAngle(const WorldImpl& world, JointID id)
{
    return GetReferenceAngle(world.GetJoint(id));
}

UnitVec GetLocalAxisA(const WorldImpl& world, JointID id)
{
    return GetLocalAxisA(world.GetJoint(id));
}

bool IsMotorEnabled(const WorldImpl& world, JointID id)
{
    Optional<bool> result;
    const auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.IsMotorEnabled();
    };
    visitor.get<const PrismaticJoint&>() = [&result](const PrismaticJoint& j) {
        result = j.IsMotorEnabled();
    };
    visitor.get<const WheelJoint&>() = [&result](const WheelJoint& j) {
        result = j.IsMotorEnabled();
    };
    joint.Accept(visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("IsMotorEnabled not supported by joint type!");
    }
    return *result;
}

void EnableMotor(WorldImpl& world, JointID id, bool value)
{
    auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<RevoluteJoint&>() = [&world,id,value](RevoluteJoint& j) {
        if (j.EnableMotor(value)) SetAwake(world, id);
    };
    visitor.get<PrismaticJoint&>() = [&world,id,value](PrismaticJoint& j) {
        if (j.EnableMotor(value)) SetAwake(world, id);
    };
    visitor.get<WheelJoint&>() = [&world,id,value](WheelJoint& j) {
        if (j.EnableMotor(value)) SetAwake(world, id);
    };
    visitor.fallback = []() {
        throw std::invalid_argument("EnableMotor not supported by joint type!");
    };
    joint.Accept(visitor);
}

AngularVelocity GetMotorSpeed(const WorldImpl& world, JointID id)
{
    Optional<AngularVelocity> result;
    const auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.GetMotorSpeed();
    };
    visitor.get<const PrismaticJoint&>() = [&result](const PrismaticJoint& j) {
        result = j.GetMotorSpeed();
    };
    visitor.get<const WheelJoint&>() = [&result](const WheelJoint& j) {
        result = j.GetMotorSpeed();
    };
    joint.Accept(visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetMotorSpeed not supported by joint type!");
    }
    return *result;
}

template <typename T>
void SetMotorSpeed(WorldImpl& world, T& j, JointID id, AngularVelocity value)
{
    if (j.GetMotorSpeed() != value)
    {
        j.SetMotorSpeed(value);
        SetAwake(world, id);
    }
}

void SetMotorSpeed(WorldImpl& world, JointID id, AngularVelocity value)
{
    auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<RevoluteJoint&>() = [&world,id,value](RevoluteJoint& j) {
        SetMotorSpeed(world, j, id, value);
    };
    visitor.get<PrismaticJoint&>() = [&world,id,value](PrismaticJoint& j) {
        SetMotorSpeed(world, j, id, value);
    };
    visitor.get<WheelJoint&>() = [&world,id,value](WheelJoint& j) {
        SetMotorSpeed(world, j, id, value);
    };
    visitor.fallback = []() {
        throw std::invalid_argument("SetMotorSpeed not supported by joint type!");
    };
    joint.Accept(visitor);
}

Torque GetMaxMotorTorque(const WorldImpl& world, JointID id)
{
    Optional<Torque> result;
    const auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.GetMaxMotorTorque();
    };
    joint.Accept(visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetMaxMotorTorque not supported by joint type!");
    }
    return *result;
}

void SetMaxMotorTorque(WorldImpl& world, JointID id, Torque value)
{
    auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<RevoluteJoint&>() = [&world,id,value](RevoluteJoint& j) {
        if (j.GetMaxMotorTorque() != value)
        {
            j.SetMaxMotorTorque(value);
            SetAwake(world, id);
        }
    };
    visitor.fallback = []() {
        throw std::invalid_argument("SetMaxMotorTorque not supported by joint type!");
    };
    joint.Accept(visitor);
}

AngularMomentum GetAngularMotorImpulse(const WorldImpl& world, JointID id)
{
    Optional<AngularMomentum> result;
    const auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.GetAngularMotorImpulse();
    };
    joint.Accept(visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetAngularMotorImpulse not supported by joint type!");
    }
    return *result;
}

RotInertia GetAngularMass(const WorldImpl& world, JointID id)
{
    Optional<RotInertia> result;
    const auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<const FrictionJoint&>() = [&result](const FrictionJoint& j) {
        result = j.GetAngularMass();
    };
    visitor.get<const MotorJoint&>() = [&result](const MotorJoint& j) {
        result = j.GetAngularMass();
    };
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.GetAngularMass();
    };
    visitor.get<const WheelJoint&>() = [&result](const WheelJoint& j) {
        result = j.GetAngularMass();
    };
    joint.Accept(visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetAngularMotorImpulse not supported by joint type!");
    }
    return *result;
}

Frequency GetFrequency(const WorldImpl& world, JointID id)
{
    Optional<Frequency> result;
    const auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<const DistanceJoint&>() = [&result](const DistanceJoint& j) {
        result = j.GetFrequency();
    };
    visitor.get<const TargetJoint&>() = [&result](const TargetJoint& j) {
        result = j.GetFrequency();
    };
    visitor.get<const WeldJoint&>() = [&result](const WeldJoint& j) {
        result = j.GetFrequency();
    };
    visitor.get<const WheelJoint&>() = [&result](const WheelJoint& j) {
        result = j.GetFrequency();
    };
    joint.Accept(visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetFrequency not supported by joint type!");
    }
    return *result;
}

void SetFrequency(WorldImpl& world, JointID id, Frequency value)
{
    auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<DistanceJoint&>() = [value](DistanceJoint& j) {
        j.SetFrequency(value);
    };
    visitor.get<TargetJoint&>() = [value](TargetJoint& j) {
        j.SetFrequency(value);
    };
    visitor.get<WeldJoint&>() = [value](WeldJoint& j) {
        j.SetFrequency(value);
    };
    visitor.get<WheelJoint&>() = [value](WheelJoint& j) {
        j.SetFrequency(value);
    };
    visitor.fallback = []() {
        throw std::invalid_argument("SetFrequency not supported by joint type!");
    };
    joint.Accept(visitor);
}

void Accept(const WorldImpl& world, JointID id, JointVisitor& visitor)
{
    world.GetJoint(id).Accept(visitor);
}

Length2 GetTarget(const WorldImpl& world, JointID id)
{
    Optional<Length2> result;
    FunctionalJointVisitor visitor;
    visitor.get<const TargetJoint&>() = [&result](const TargetJoint& j) {
        result = j.GetTarget();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetTarget not supported by joint type!");
    }
    return *result;
}

void SetTarget(WorldImpl& world, JointID id, Length2 value)
{
    auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<TargetJoint&>() = [value,&world](TargetJoint& j) {
        if (j.GetTarget() != value)
        {
            j.SetTarget(value);
            world.GetBody(j.GetBodyB()).SetAwake();
        }
    };
    visitor.fallback = []() {
        throw std::invalid_argument("SetTarget not supported by joint type!");
    };
    joint.Accept(visitor);
}

Angle GetAngularLowerLimit(const WorldImpl& world, JointID id)
{
    Optional<Angle> result;
    FunctionalJointVisitor visitor;
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.GetAngularLowerLimit();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetAngularLowerLimit not supported by joint type!");
    }
    return *result;
}

Angle GetAngularUpperLimit(const WorldImpl& world, JointID id)
{
    Optional<Angle> result;
    FunctionalJointVisitor visitor;
    visitor.get<const RevoluteJoint&>() = [&result](const RevoluteJoint& j) {
        result = j.GetAngularUpperLimit();
    };
    Accept(world, id, visitor);
    if (!result.has_value())
    {
        throw std::invalid_argument("GetAngularUpperLimit not supported by joint type!");
    }
    return *result;
}

void SetAngularLimits(WorldImpl& world, JointID id, Angle lower, Angle upper)
{
    auto& joint = world.GetJoint(id);
    FunctionalJointVisitor visitor;
    visitor.get<RevoluteJoint&>() = [&world,lower,upper](RevoluteJoint& j) {
        if (j.GetAngularLowerLimit() != lower || j.GetAngularUpperLimit() != upper)
        {
            j.SetAngularLimits(lower, upper);
            world.GetBody(j.GetBodyA()).SetAwake();
            world.GetBody(j.GetBodyB()).SetAwake();
        }
    };
    visitor.fallback = []() {
        throw std::invalid_argument("SetAngularLimits not supported by joint type!");
    };
    joint.Accept(visitor);
}

} // namespace d2
} // namespace playrho