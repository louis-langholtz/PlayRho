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

#ifndef PLAYRHO_DYNAMICS_JOINTS_MOUSEJOINTDEF_HPP
#define PLAYRHO_DYNAMICS_JOINTS_MOUSEJOINTDEF_HPP

#include <PlayRho/Dynamics/Joints/JointDef.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

class MouseJoint;
class Body;

/// @brief Mouse joint definition.
/// @details This requires a world target point, tuning parameters, and the time step.
struct MouseJointDef : public JointBuilder<MouseJointDef>
{
    /// @brief Super type.
    using super = JointBuilder<MouseJointDef>;
    
    constexpr MouseJointDef() noexcept: super{JointType::Mouse} {}

    /// @brief Initializing constructor.
    constexpr MouseJointDef(NonNull<Body*> b) noexcept: super{super{JointType::Mouse}.UseBodyB(b)}
    {
        // Intentionally empty.
    }
    
    /// @brief Use value for target.
    constexpr MouseJointDef& UseTarget(Length2D v) noexcept;

    /// @brief Use value for max force.
    constexpr MouseJointDef& UseMaxForce(NonNegative<Force> v) noexcept;

    /// @brief Use value for frequency.
    constexpr MouseJointDef& UseFrequency(NonNegative<Frequency> v) noexcept;

    /// @brief Use value for damping ratio.
    constexpr MouseJointDef& UseDampingRatio(NonNegative<Real> v) noexcept;

    /// The initial world target point. This is assumed
    /// to coincide with the body anchor initially.
    Length2D target = Length2D{};
    
    /// Max force.
    /// @details
    /// The maximum constraint force that can be exerted
    /// to move the candidate body. Usually you will express
    /// as some multiple of the weight (multiplier * mass * gravity).
    /// @note This may not be negative.
    NonNegative<Force> maxForce = NonNegative<Force>{0};
    
    /// Frequency.
    /// @details The has to do with the response speed.
    /// @note This value may not be negative.
    NonNegative<Frequency> frequency = NonNegative<Frequency>(Real{5} * Hertz);
    
    /// The damping ratio. 0 = no damping, 1 = critical damping.
    NonNegative<Real> dampingRatio = NonNegative<Real>(0.7f);
};

constexpr MouseJointDef& MouseJointDef::UseTarget(Length2D v) noexcept
{
    target = v;
    return *this;
}

constexpr MouseJointDef& MouseJointDef::UseMaxForce(NonNegative<Force> v) noexcept
{
    maxForce = v;
    return *this;
}

constexpr MouseJointDef& MouseJointDef::UseFrequency(NonNegative<Frequency> v) noexcept
{
    frequency = v;
    return *this;
}

constexpr MouseJointDef& MouseJointDef::UseDampingRatio(NonNegative<Real> v) noexcept
{
    dampingRatio = v;
    return *this;
}

/// @brief Gets the definition data for the given joint.
/// @relatedalso MouseJoint
MouseJointDef GetMouseJointDef(const MouseJoint& joint) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_MOUSEJOINTDEF_HPP
