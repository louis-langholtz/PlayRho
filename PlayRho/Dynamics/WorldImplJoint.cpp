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
#include <PlayRho/Dynamics/Joints/RevoluteJointConf.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJointConf.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJointConf.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJointConf.hpp>
#include <PlayRho/Dynamics/Joints/TargetJointConf.hpp>
#include <PlayRho/Dynamics/Joints/GearJointConf.hpp>
#include <PlayRho/Dynamics/Joints/WheelJointConf.hpp>
#include <PlayRho/Dynamics/Joints/WeldJointConf.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJointConf.hpp>
#include <PlayRho/Dynamics/Joints/RopeJointConf.hpp>
#include <PlayRho/Dynamics/Joints/MotorJointConf.hpp>

#include <PlayRho/Common/OptionalValue.hpp> // for Optional

namespace playrho {
namespace d2 {

JointID CreateJoint(WorldImpl& world, const Joint& def)
{
    return world.CreateJoint(def);
}

void Destroy(WorldImpl& world, JointID id)
{
    world.Destroy(id);
}

const Joint& GetJoint(const WorldImpl& world, JointID id)
{
    return world.GetJoint(id);
}

void SetJoint(WorldImpl& world, JointID id, const Joint& def)
{
    world.SetJoint(id, def);
}

JointType GetType(const WorldImpl& world, JointID id)
{
    return ::playrho::d2::GetType(world.GetJoint(id));
}

const void* GetData(const WorldImpl& world, JointID id)
{
    return &world.GetJoint(id);
}

Momentum2 GetLinearReaction(const WorldImpl& world, JointID id)
{
    return GetLinearReaction(world.GetJoint(id));
}

AngularMomentum GetAngularReaction(const WorldImpl& world, JointID id)
{
    return GetAngularReaction(world.GetJoint(id));
}

void SetAwake(WorldImpl& world, JointID id)
{
    const auto& joint = world.GetJoint(id);
    const auto bA = GetBodyA(joint);
    const auto bB = GetBodyB(joint);
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
    return GetCollideConnected(world.GetJoint(id));
}

void* GetUserData(const WorldImpl& world, JointID id)
{
    return GetUserData(world.GetJoint(id));
}

void SetUserData(WorldImpl& world, JointID id, void* value)
{
    SetUserData(world.GetJoint(id), value);
}

BodyID GetBodyA(const WorldImpl& world, JointID id)
{
    return GetBodyA(world.GetJoint(id));
}

BodyID GetBodyB(const WorldImpl& world, JointID id)
{
    return GetBodyB(world.GetJoint(id));
}

Length2 GetLocalAnchorA(const WorldImpl& world, JointID id)
{
    return GetLocalAnchorA(world.GetJoint(id));
}

Length2 GetLocalAnchorB(const WorldImpl& world, JointID id)
{
    return GetLocalAnchorB(world.GetJoint(id));
}

Angle GetReferenceAngle(const WorldImpl& world, JointID id)
{
    return GetReferenceAngle(world.GetJoint(id));
}

} // namespace d2
} // namespace playrho
