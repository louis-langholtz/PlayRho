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

#ifndef PLAYRHO_DYNAMICS_JOINTS_FRICTIONJOINT_HPP
#define PLAYRHO_DYNAMICS_JOINTS_FRICTIONJOINT_HPP

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/FrictionJointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>

namespace playrho {

/// @brief Friction joint.
///
/// @details This is used for top-down friction. It provides 2D translational friction
///   and angular friction.
///
/// @ingroup JointsGroup
///
class FrictionJoint : public Joint
{
public:
    
    /// @brief Initializing constructor.
    FrictionJoint(const FrictionJointDef& def);

    void Accept(JointVisitor& visitor) const override;

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Momentum2D GetLinearReaction() const override;
    AngularMomentum GetAngularReaction() const override;

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

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                 const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    NonNegative<Force> m_maxForce = NonNegative<Force>{0};
    NonNegative<Torque> m_maxTorque = NonNegative<Torque>{0};

    // Solver shared data - data saved & updated over multiple InitVelocityConstraints calls.
    Momentum2D m_linearImpulse = Momentum2D{};
    AngularMomentum m_angularImpulse = AngularMomentum{0};

    // Solver temp
    Length2D m_rA;
    Length2D m_rB;
    Mass22 m_linearMass; ///< 2x2 linear mass matrix in kilograms.
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

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_FRICTIONJOINT_HPP
