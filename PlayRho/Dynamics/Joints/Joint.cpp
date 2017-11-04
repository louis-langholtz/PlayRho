/*
 * Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/Joints/WheelJoint.hpp>
#include <PlayRho/Dynamics/Joints/MouseJoint.hpp>
#include <PlayRho/Dynamics/Joints/RevoluteJoint.hpp>
#include <PlayRho/Dynamics/Joints/PrismaticJoint.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJoint.hpp>
#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Dynamics/Joints/WeldJoint.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJoint.hpp>
#include <PlayRho/Dynamics/Joints/RopeJoint.hpp>
#include <PlayRho/Dynamics/Joints/MotorJoint.hpp>
#include <PlayRho/Dynamics/Body.hpp>
#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Dynamics/Contacts/Contact.hpp>
#include <PlayRho/Defines.hpp>

#include <algorithm>

namespace playrho {

Joint* Joint::Create(const JointDef& def)
{
    assert(def.type != JointType::Unknown);

    switch (def.type)
    {
        case JointType::Distance:
            return Create<DistanceJoint>(static_cast<const DistanceJointDef&>(def));
        case JointType::Mouse:
            return Create<MouseJoint>(static_cast<const MouseJointDef&>(def));
        case JointType::Prismatic:
            return Create<PrismaticJoint>(static_cast<const PrismaticJointDef&>(def));
        case JointType::Revolute:
            return Create<RevoluteJoint>(static_cast<const RevoluteJointDef&>(def));
        case JointType::Pulley:
            return Create<PulleyJoint>(static_cast<const PulleyJointDef&>(def));
        case JointType::Gear:
            return Create<GearJoint>(static_cast<const GearJointDef&>(def));
        case JointType::Wheel:
            return Create<WheelJoint>(static_cast<const WheelJointDef&>(def));
        case JointType::Weld:
            return Create<WeldJoint>(static_cast<const WeldJointDef&>(def));
        case JointType::Friction:
            return Create<FrictionJoint>(static_cast<const FrictionJointDef&>(def));
        case JointType::Rope:
            return Create<RopeJoint>(static_cast<const RopeJointDef&>(def));
        case JointType::Motor:
            return Create<MotorJoint>(static_cast<const MotorJointDef&>(def));
        case JointType::Unknown:
            throw InvalidArgument("Joint::Create: Unknown joint type");
    }

    PLAYRHO_UNREACHABLE;
}

Joint::FlagsType Joint::GetFlags(const JointDef& def) noexcept
{
    auto flags = Joint::FlagsType(0);
    if (def.collideConnected)
    {
        flags |= e_collideConnectedFlag;
    }
    return flags;
}

Joint::Joint(const JointDef& def):
    m_bodyA{def.bodyA}, m_bodyB{def.bodyB}, m_flags{GetFlags(def)}, m_userData{def.userData}
{
    // Intentionally empty.
}

void Joint::Destroy(const Joint* joint)
{
    delete joint;
}

bool Joint::IsOkay(const JointDef& def) noexcept
{
    return def.bodyA != def.bodyB;
}

// Free functions...

bool IsEnabled(const Joint& j) noexcept
{
    const auto bA = j.GetBodyA();
    const auto bB = j.GetBodyB();
    return ((bA == nullptr) || bA->IsEnabled()) && ((bB == nullptr) || bB->IsEnabled());
}

void SetAwake(Joint& j) noexcept
{
    const auto bA = j.GetBodyA();
    const auto bB = j.GetBodyB();
    if (bA != nullptr)
    {
        bA->SetAwake();
    }
    if (bB != nullptr)
    {
        bB->SetAwake();
    }
}

JointCounter GetWorldIndex(const Joint* joint)
{
    if (joint != nullptr)
    {
        const auto bA = joint->GetBodyA();
        const auto bB = joint->GetBodyB();
        const auto world = (bA != nullptr)? bA->GetWorld():
            (bB != nullptr)? bB->GetWorld(): static_cast<const World*>(nullptr);
        if (world != nullptr)
        {
            auto i = JointCounter{0};
            const auto joints = world->GetJoints();
            const auto it = std::find_if(std::cbegin(joints), std::cend(joints), [&](const Joint *j) {
                return (j == joint) || ((void) ++i, false);
            });
            if (it != std::end(joints))
            {
                return i;
            }
        }
    }
    return JointCounter(-1);
}

#ifdef PLAYRHO_PROVIDE_VECTOR_AT
BodyConstraintPtr& At(std::vector<BodyConstraintPair>& container, const Body* key)
{
    auto last = std::end(container);
    auto first = std::begin(container);
    first = std::lower_bound(first, last, key, [](const BodyConstraintPair &a, const Body* b){
        return a.first < b;
    });
    if (first == last || key != (*first).first)
    {
        throw std::out_of_range{"invalid key"};
    }
    return (*first).second;
}
#endif

BodyConstraintPtr& At(std::unordered_map<const Body*, BodyConstraint*>& container,
                      const Body* key)
{
    return container.at(key);
}

const char* ToString(Joint::LimitState val) noexcept
{
    switch (val)
    {
        case Joint::e_inactiveLimit: return "inactive";
        case Joint::e_atLowerLimit: return "at lower";
        case Joint::e_atUpperLimit: return "at upper";
        case Joint::e_equalLimits: return "equal";
    }
    return "unknown";
}

} // namespace playrho
