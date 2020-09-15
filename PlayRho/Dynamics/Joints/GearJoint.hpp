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

#ifndef PLAYRHO_DYNAMICS_JOINTS_GEARJOINT_HPP
#define PLAYRHO_DYNAMICS_JOINTS_GEARJOINT_HPP

#include <PlayRho/Dynamics/Joints/Joint.hpp>

#include <PlayRho/Dynamics/Joints/JointID.hpp>
#include <PlayRho/Dynamics/Joints/GearJointConf.hpp>

namespace playrho {
namespace d2 {

/// @brief Gear joint.
///
/// @details A gear joint is used to connect two joints together. Either joint can be
///   a revolute or prismatic joint. You specify a gear ratio to bind the motions together:
///      <code>coordinate1 + ratio * coordinate2 = constant</code>.
///   The ratio can be negative or positive. If one joint is a revolute joint and the other
///   joint is a prismatic joint, then the ratio will have units of length or units of 1/length.
///
/// @warning You have to manually destroy the gear joint if joint-1 or joint-2 is destroyed.
///
/// @ingroup JointsGroup
///
/// @image html gearJoint.gif
///
class GearJoint : public Joint
{
public:
    /// @brief Is the given definition okay.
    static bool IsOkay(const GearJointConf& data) noexcept;

    /// @brief Initializing constructor.
    /// @attention To create or use the joint within a world instance, call that world
    ///   instance's create joint method instead of calling this constructor directly.
    /// @see World::CreateJoint
    GearJoint(const GearJointConf& data);

    void Accept(JointVisitor& visitor) const override;
    void Accept(JointVisitor& visitor) override;

    Length2 GetLocalAnchorA() const noexcept override { return m_localAnchorA; }
    Length2 GetLocalAnchorB() const noexcept override { return m_localAnchorB; }

    Momentum2 GetLinearReaction() const override;
    AngularMomentum GetAngularReaction() const override;

    /// @brief Gets the local anchor point relative to body C's origin.
    Length2 GetLocalAnchorC() const noexcept { return m_localAnchorC; }

    /// @brief Gets the local anchor point relative to body D's origin.
    Length2 GetLocalAnchorD() const noexcept { return m_localAnchorD; }

    /// @brief Gets the first joint type.
    JointType GetType1() const noexcept { return m_type1; }

    /// @brief Gets the second joint type.
    JointType GetType2() const noexcept { return m_type2; }

    BodyID GetBodyC() const noexcept { return m_bodyC; }
    BodyID GetBodyD() const noexcept { return m_bodyD; }

    UnitVec GetLocalAxis1() const noexcept { return m_localAxis1; }
    UnitVec GetLocalAxis2() const noexcept { return m_localAxis2; }
    Angle GetReferenceAngle1() const noexcept { return m_referenceAngle1; }
    Angle GetReferenceAngle2() const noexcept { return m_referenceAngle2; }

    /// @brief Sets the gear ratio.
    void SetRatio(Real ratio);

    /// @brief Gets the ratio for position solving.
    Real GetRatio() const noexcept;

    /// @brief Gets the constant for position solving.
    Real GetConstant() const noexcept;

private:
    void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                 const ConstraintSolverConf& conf) override;
    bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) override;
    bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                  const ConstraintSolverConf& conf) const override;

    JointType m_type1; ///< Type of joint 1.
    JointType m_type2; ///< Type of joint 2.

    // Body A is connected to body C
    // Body B is connected to body D
    BodyID m_bodyC; ///< Body C.
    BodyID m_bodyD; ///< Body D.

    // Solver shared

    // Local anchors used when type1 is not revolute
    Length2 m_localAnchorA; ///< Local anchor A.
    Length2 m_localAnchorC; ///< Local anchor C.

    // Local anchors used when type2 is not revolute
    Length2 m_localAnchorB; ///< Local anchor B.
    Length2 m_localAnchorD; ///< Local anchor D.

    UnitVec m_localAxis1; ///< Local axis 1. Used when type1 is not Revolute.
    UnitVec m_localAxis2; ///< Local axis 2. Used when type2 is not Revolute.

    Angle m_referenceAngle1; ///< Reference angle of joint 1. Used when type1 is Revolute.
    Angle m_referenceAngle2; ///< Reference angle of joint 2. Used when type2 is Revolute.

    Real m_ratio; ///< Ratio for position solving.
    Real m_constant; ///< Constant for position solving.

    Momentum m_impulse = 0_Ns; ///< Impulse.

    // Solver temp
    Vec2 m_JvAC = Vec2{}; ///< <code>AC Jv</code> data.
    Vec2 m_JvBD; ///< <code>BD Jv</code> data.
    Length m_JwA = 0_m; ///< A <code>Jw</code> data.
    Length m_JwB; ///< B <code>Jw</code> data.
    Length m_JwC; ///< C <code>Jw</code> data.
    Length m_JwD; ///< D <code>Jw</code> data.
    Real m_mass; ///< Either linear mass or angular mass.
};

inline Real GearJoint::GetRatio() const noexcept
{
    return m_ratio;
}

inline Real GearJoint::GetConstant() const noexcept
{
    return m_constant;
}

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_GEARJOINT_HPP
