/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef B2_FRICTION_JOINT_H
#define B2_FRICTION_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho {

/// @brief Friction joint definition.
struct FrictionJointDef : public JointBuilder<FrictionJointDef>
{
    using super = JointBuilder<FrictionJointDef>;

    constexpr FrictionJointDef() noexcept: super{JointType::Friction} {}

    /// @brief Initializing constructor.
    /// @details Initialize the bodies, anchors, axis, and reference angle using the world
    ///   anchor and world axis.
    FrictionJointDef(Body* bodyA, Body* bodyB, const Length2D anchor) noexcept;

    FrictionJointDef& UseMaxForce(NonNegative<Force> v) noexcept
    {
        maxForce = v;
        return *this;
    }
    
    FrictionJointDef& UseMaxTorque(NonNegative<Torque> v) noexcept
    {
        maxTorque = v;
        return *this;
    }

    /// @brief Local anchor point relative to bodyA's origin.
    Length2D localAnchorA = Length2D(0, 0);

    /// @brief Local anchor point relative to bodyB's origin.
    Length2D localAnchorB = Length2D(0, 0);

    /// @brief Maximum friction force.
    NonNegative<Force> maxForce = NonNegative<Force>{0};

    /// @brief Maximum friction torque.
    NonNegative<Torque> maxTorque = NonNegative<Torque>{0};
};

/// @brief Friction joint.
/// @details This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
class FrictionJoint : public Joint
{
public:
    FrictionJoint(const FrictionJointDef& def);

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Force2D GetReactionForce(Frequency inv_dt) const override;
    Torque GetReactionTorque(Frequency inv_dt) const override;

    /// The local anchor point relative to bodyA's origin.
    Length2D GetLocalAnchorA() const { return m_localAnchorA; }

    /// The local anchor point relative to bodyB's origin.
    Length2D GetLocalAnchorB() const  { return m_localAnchorB; }

    /// Set the maximum friction force in N.
    void SetMaxForce(NonNegative<Force> force);

    /// Get the maximum friction force in N.
    NonNegative<Force> GetMaxForce() const;

    /// Set the maximum friction torque in N*m.
    void SetMaxTorque(NonNegative<Torque> torque);

    /// Get the maximum friction torque in N*m.
    NonNegative<Torque> GetMaxTorque() const;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step, const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const override;

    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    NonNegative<Force> m_maxForce;
    NonNegative<Torque> m_maxTorque;

    // Solver shared data - data saved & updated over multiple InitVelocityConstraints calls.
    Momentum2D m_linearImpulse = Momentum2D{0, 0};
    AngularMomentum m_angularImpulse = AngularMomentum{0};

    // Solver temp
    Length2D m_rA;
    Length2D m_rB;
    Mat22 m_linearMass; ///< 2x2 linear mass matrix in kilograms.
    RotInertia m_angularMass;
};

inline void FrictionJoint::SetMaxForce(NonNegative<Force> force)
{
    m_maxForce = force;
}

inline NonNegative<Force> FrictionJoint::GetMaxForce() const
{
    return m_maxForce;
}

inline void FrictionJoint::SetMaxTorque(NonNegative<Torque> torque)
{
    m_maxTorque = torque;
}

inline NonNegative<Torque> FrictionJoint::GetMaxTorque() const
{
    return m_maxTorque;
}

FrictionJointDef GetFrictionJointDef(const FrictionJoint& joint) noexcept;

} // namespace playrho

#endif
