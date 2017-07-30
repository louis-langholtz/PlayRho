/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <PlayRho/Dynamics/Joints/DistanceJointDef.hpp>
#include <PlayRho/Dynamics/Joints/DistanceJoint.hpp>
#include <PlayRho/Dynamics/Body.hpp>

using namespace playrho;

DistanceJointDef::DistanceJointDef(NonNull<Body*> bA, NonNull<Body*> bB,
                                   Length2D anchor1, Length2D anchor2) noexcept :
    super{super{JointType::Distance}.UseBodyA(bA).UseBodyB(bB)},
    localAnchorA{GetLocalPoint(*bA, anchor1)},
    localAnchorB{GetLocalPoint(*bB, anchor2)},
    length{GetLength(anchor2 - anchor1)}
{
    // Intentionally empty.
}

DistanceJointDef playrho::GetDistanceJointDef(const DistanceJoint& joint) noexcept
{
    auto def = DistanceJointDef{};
    
    Set(def, joint);
    
    def.localAnchorA = joint.GetLocalAnchorA();
    def.localAnchorB = joint.GetLocalAnchorB();
    def.length = joint.GetLength();
    def.frequency = joint.GetFrequency();
    def.dampingRatio = joint.GetDampingRatio();
    
    return def;
}