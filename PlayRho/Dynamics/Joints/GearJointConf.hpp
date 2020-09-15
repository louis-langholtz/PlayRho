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

#ifndef PLAYRHO_DYNAMICS_JOINTS_GEARJOINTCONF_HPP
#define PLAYRHO_DYNAMICS_JOINTS_GEARJOINTCONF_HPP

#include <PlayRho/Dynamics/Joints/JointConf.hpp>

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Dynamics/Joints/JointID.hpp>

namespace playrho {
namespace d2 {

class Joint;
class GearJoint;
class World;

/// @brief Gear joint definition.
/// @details This definition requires two existing
/// revolute or prismatic joints (any combination will work).
struct GearJointConf : public JointBuilder<GearJointConf>
{
    /// @brief Super type.
    using super = JointBuilder<GearJointConf>;

    /// @brief Initializing constructor.
    GearJointConf(BodyID bA, BodyID bB, BodyID bC, BodyID bD) noexcept;

    /// @brief Uses the given ratio value.
    GearJointConf& UseRatio(Real v) noexcept;

    BodyID bodyC = InvalidBodyID;
    BodyID bodyD = InvalidBodyID;

    JointType type1 = JointType::Unknown;
    JointType type2 = JointType::Unknown;

    // Used when not Revolute...
    Length2 localAnchorA{}; ///< Local anchor A.
    Length2 localAnchorB{}; ///< Local anchor B.
    Length2 localAnchorC{}; ///< Local anchor C.
    Length2 localAnchorD{}; ///< Local anchor D.
    
    UnitVec localAxis1; ///< Local axis 1. Used when type1 is not Revolute.
    UnitVec localAxis2; ///< Local axis 2. Used when type2 is not Revolute.

    Angle referenceAngle1; ///< Reference angle of joint 1. Used when type1 is Revolute.
    Angle referenceAngle2; ///< Reference angle of joint 2. Used when type2 is Revolute.

    /// The gear ratio.
    /// @see GearJoint for explanation.
    Real ratio = Real{1};

    Real constant = Real{0};
};

inline GearJointConf& GearJointConf::UseRatio(Real v) noexcept
{
    ratio = v;
    return *this;
}

/// @brief Gets the definition data for the given joint.
/// @relatedalso GearJoint
GearJointConf GetGearJointConf(const GearJoint& joint) noexcept;

GearJointConf GetGearJointConf(const World& world, JointID id1, JointID id2, Real ratio = Real{1});

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_GEARJOINTCONF_HPP
