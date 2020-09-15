/*
 * Original work Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

#include <PlayRho/Dynamics/Joints/GearJointConf.hpp>

#include <PlayRho/Dynamics/Joints/GearJoint.hpp>
#include <PlayRho/Dynamics/World.hpp>

namespace playrho {
namespace d2 {

GearJointConf::GearJointConf(BodyID bA, BodyID bB, BodyID bC, BodyID bD) noexcept:
    super{super{JointType::Gear}.UseBodyA(bA).UseBodyB(bB)},
    bodyC(bC), bodyD(bD)
{
    // Intentionally empty.
}

GearJointConf GetGearJointConf(const GearJoint& joint) noexcept
{
    auto def = GearJointConf{joint.GetBodyA(), joint.GetBodyB(), joint.GetBodyC(), joint.GetBodyD()};
    Set(def, joint);
    def.localAnchorA = joint.GetLocalAnchorA();
    def.localAnchorB = joint.GetLocalAnchorB();
    def.localAnchorC = joint.GetLocalAnchorC();
    def.localAnchorD = joint.GetLocalAnchorD();
    def.localAxis1 = joint.GetLocalAxis1();
    def.localAxis2 = joint.GetLocalAxis2();
    def.referenceAngle1 = joint.GetReferenceAngle1();
    def.referenceAngle2 = joint.GetReferenceAngle2();
    def.ratio = joint.GetRatio();
    def.constant = joint.GetConstant();
    return def;
}

GearJointConf GetGearJointConf(const World& world, JointID id1, JointID id2, Real ratio)
{
    auto def = GearJointConf{
        GetBodyB(world, id1), GetBodyB(world, id2),
        GetBodyA(world, id1), GetBodyA(world, id2)
    };

    auto coordinateA = Real{0};
    def.type1 = GetType(world, id1);
    switch (def.type1)
    {
        case JointType::Revolute:
        {
            def.referenceAngle1 = GetReferenceAngle(world, id1);
            coordinateA = (GetAngle(world, def.bodyA) - GetAngle(world, def.bodyC) - def.referenceAngle1) / Radian;
            break;
        }
        case JointType::Prismatic:
        {
            const auto xfA = GetTransformation(world, def.bodyA);
            const auto xfC = GetTransformation(world, def.bodyC);
            def.localAnchorC = GetLocalAnchorA(world, id1);
            def.localAnchorA = GetLocalAnchorB(world, id1);
            def.localAxis1 = GetLocalAxisA(world, id1);
            const auto pC = def.localAnchorC;
            const auto pA = InverseRotate(Rotate(def.localAnchorA, xfA.q) + (xfA.p - xfC.p), xfC.q);
            coordinateA = Dot(pA - pC, def.localAxis1) / Meter;
            break;
        }
        default:
            break;
    }

    auto coordinateB = Real{0};
    def.type2 = GetType(world, id2);
    switch (def.type2)
    {
        case JointType::Revolute:
        {
            def.referenceAngle2 = GetReferenceAngle(world, id2);
            coordinateA = (GetAngle(world, def.bodyB) - GetAngle(world, def.bodyD) - def.referenceAngle1) / Radian;
            break;
        }
        case JointType::Prismatic:
        {
            const auto xfB = GetTransformation(world, def.bodyB);
            const auto xfD = GetTransformation(world, def.bodyD);
            def.localAnchorD = GetLocalAnchorA(world, id2);
            def.localAnchorB = GetLocalAnchorB(world, id2);
            def.localAxis2 = GetLocalAxisA(world, id2);
            const auto pD = def.localAnchorD;
            const auto pB = InverseRotate(Rotate(def.localAnchorB, xfB.q) + (xfB.p - xfD.p), xfD.q);
            coordinateB = Dot(pB - pD, def.localAxis2) / Meter;
            break;
        }
        default:
            break;
    }

    def.ratio = ratio;
    def.constant = coordinateA + def.ratio * coordinateB;

    return def;
}

} // namespace d2
} // namespace playrho
