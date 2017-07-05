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

#ifndef B2_ROPE_JOINT_H
#define B2_ROPE_JOINT_H

#include <Box2D/Dynamics/Joints/Joint.hpp>

namespace box2d {

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in JointDef.
struct RopeJointDef : public JointDef
{
    constexpr RopeJointDef() noexcept: JointDef(JointType::Rope) {}

    constexpr RopeJointDef(Body* bodyA, Body* bodyB) noexcept: JointDef(JointType::Rope, bodyA, bodyB) {}

    /// The local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D{RealNum(-1) * Meter, RealNum(0) * Meter};

    /// The local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D{RealNum(1) * Meter, RealNum(0) * Meter};

    /// The maximum length of the rope.
    Length maxLength = Length{0};
};

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See DistanceJoint if you want to dynamically
/// control length.
class RopeJoint : public Joint
{
public:
    RopeJoint(const RopeJointDef& data);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Force2D GetReactionForce(Frequency inv_dt) const override;
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const  { return m_localAnchorB; }

    /// Set/Get the maximum length of the rope.
    void SetMaxLength(Length length) { m_maxLength = length; }
    Length GetMaxLength() const;

    LimitState GetLimitState() const;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const override;

    // Solver shared
    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Length m_maxLength;
    Length m_length = 0;
    Momentum m_impulse = Momentum{0};

    // Solver temp
    UnitVec2 m_u;
    Length2D m_rA;
    Length2D m_rB;
    Mass m_mass = Mass{0};
    LimitState m_state = e_inactiveLimit;
};

RopeJointDef GetRopeJointDef(const RopeJoint& joint) noexcept;

} // namespace box2d

#endif
