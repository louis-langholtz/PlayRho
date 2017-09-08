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

#ifndef B2_PULLEY_JOINT_H
#define B2_PULLEY_JOINT_H

#include <PlayRho/Dynamics/Joints/Joint.hpp>
#include <PlayRho/Dynamics/Joints/PulleyJointDef.hpp>

namespace playrho {

/// @brief Pulley joint.
/// @details The pulley joint is connected to two bodies and two fixed ground points.
///   The pulley supports a ratio such that: length1 + ratio * length2 <= constant.
/// @note The force transmitted is scaled by the ratio.
/// @warning the pulley joint can get a bit squirrelly by itself. They often
///   work better when combined with prismatic joints. You should also cover the
///   the anchor points with static shapes to prevent one side from going to
///   zero length.
class PulleyJoint : public Joint
{
public:
    
    /// @brief Initializing constructor.
    PulleyJoint(const PulleyJointDef& data);

    /// @brief Gets the local anchor A.
    Length2D GetLocalAnchorA() const noexcept;

    /// @brief Gets the local anchor B.
    Length2D GetLocalAnchorB() const noexcept;

    Length2D GetAnchorA() const override;
    Length2D GetAnchorB() const override;

    Momentum2D GetLinearReaction() const override;
    AngularMomentum GetAngularReaction() const override;

    /// Get the first ground anchor.
    Length2D GetGroundAnchorA() const noexcept;

    /// Get the second ground anchor.
    Length2D GetGroundAnchorB() const noexcept;

    /// Get the current length of the segment attached to bodyA.
    Length GetLengthA() const noexcept;

    /// Get the current length of the segment attached to bodyB.
    Length GetLengthB() const noexcept;

    /// Get the pulley ratio.
    Real GetRatio() const noexcept;

    /// Implement Joint::ShiftOrigin
    void ShiftOrigin(const Length2D newOrigin) override;

private:

    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                 const ConstraintSolverConf&) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf&) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    Length2D m_groundAnchorA;
    Length2D m_groundAnchorB;
    Length m_lengthA;
    Length m_lengthB;
    Length2D m_localAnchorA;
    Length2D m_localAnchorB;
    Real m_ratio;
    Length m_constant;
    
    // Solver shared (between calls to InitVelocityConstraints).
    Momentum m_impulse = Momentum{0};

    // Solver temp (recalculated every call to InitVelocityConstraints).
    UnitVec2 m_uA; ///< Unit vector A.
    UnitVec2 m_uB; ///< Unit vector B.
    Length2D m_rA;
    Length2D m_rB;
    Mass m_mass;
};
    
inline Length2D PulleyJoint::GetLocalAnchorA() const noexcept
{
    return m_localAnchorA;
}

inline Length2D PulleyJoint::GetLocalAnchorB() const noexcept
{
    return m_localAnchorB;
}

inline Length2D PulleyJoint::GetGroundAnchorA() const noexcept
{
    return m_groundAnchorA;
}

inline Length2D PulleyJoint::GetGroundAnchorB() const noexcept
{
    return m_groundAnchorB;
}
    
inline Length PulleyJoint::GetLengthA() const noexcept
{
    return m_lengthA;
}

inline Length PulleyJoint::GetLengthB() const noexcept
{
    return m_lengthB;
}

inline Real PulleyJoint::GetRatio() const noexcept
{
    return m_ratio;
}

/// Get the current length of the segment attached to bodyA.
Length GetCurrentLengthA(const PulleyJoint& joint);

/// Get the current length of the segment attached to bodyB.
Length GetCurrentLengthB(const PulleyJoint& joint);

} // namespace playrho

#endif
