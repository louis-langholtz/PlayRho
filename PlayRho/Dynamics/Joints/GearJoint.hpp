/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_GEAR_JOINT_H
#define B2_GEAR_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
struct GearJointDef : public JointDef
{
    constexpr GearJointDef() noexcept: JointDef(JointType::Gear) {}

    /// The first revolute/prismatic joint attached to the gear joint.
    Joint* joint1 = nullptr;

    /// The second revolute/prismatic joint attached to the gear joint.
    Joint* joint2 = nullptr;

    /// The gear ratio.
    /// @see GearJoint for explanation.
    Real ratio = Real{1};
};

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning You have to manually destroy the gear joint if joint1 or joint2
/// is destroyed.
class GearJoint : public Joint
{
public:
    static bool IsOkay(const GearJointDef& data) noexcept;

    GearJoint(const GearJointDef& data);
    
    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Force2D GetReactionForce(Frequency inv_dt) const override;
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const noexcept { return m_localAnchorA; }
    
    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const noexcept { return m_localAnchorB; }

    /// Get the first joint.
    Joint* GetJoint1() const noexcept { return m_joint1; }

    /// Get the second joint.
    Joint* GetJoint2() const noexcept { return m_joint2; }
   
    /// Set/Get the gear ratio.
    void SetRatio(Real ratio);
    Real GetRatio() const;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const override;

    Joint* m_joint1;
    Joint* m_joint2;

    JointType m_typeA;
    JointType m_typeB;

    // Body A is connected to body C
    // Body B is connected to body D
    Body* m_bodyC;
    Body* m_bodyD;

    // Solver shared
    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Length2D m_localAnchorC;
    Length2D m_localAnchorD;

    UnitVec2 m_localAxisC;
    UnitVec2 m_localAxisD;

    Angle m_referenceAngleA;
    Angle m_referenceAngleB;

    Real m_constant;
    Real m_ratio;

    Momentum m_impulse = Momentum{0};

    // Solver temp
    Vec2 m_JvAC;
    Vec2 m_JvBD;
    Length m_JwA;
    Length m_JwB;
    Length m_JwC;
    Length m_JwD;
    Real m_mass; ///< Either linear mass or angular mass.
};

GearJointDef GetGearJointDef(const GearJoint& joint) noexcept;
    
} // namespace box2d

#endif
