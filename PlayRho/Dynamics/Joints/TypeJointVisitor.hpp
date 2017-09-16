/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_DYNAMICS_JOINTS_TYPEJOINTVISITOR_HPP
#define PLAYRHO_DYNAMICS_JOINTS_TYPEJOINTVISITOR_HPP

#include <PlayRho/Dynamics/Joints/JointVisitor.hpp>
#include <PlayRho/Dynamics/Joints/JointType.hpp>
#include <PlayRho/Common/OptionalValue.hpp>

namespace playrho {

/// @brief Typing JointVisitor.
/// @details Records the type of joint that gets visited.
class TypeJointVisitor: public JointVisitor
{
public:
    
    void Visit(const RevoluteJoint& /*joint*/) override
    {
        m_type = JointType::Revolute;
    }
    
    void Visit(const PrismaticJoint& /*joint*/) override
    {
        m_type = JointType::Prismatic;
    }
    
    void Visit(const DistanceJoint& /*joint*/) override
    {
        m_type = JointType::Distance;
    }
    
    void Visit(const PulleyJoint& /*joint*/) override
    {
        m_type = JointType::Pulley;
    }
    
    /// @brief Visits a MouseJoint.
    void Visit(const MouseJoint& /*joint*/) override
    {
        m_type = JointType::Mouse;
    }
    
    /// @brief Visits a GearJoint.
    void Visit(const GearJoint& /*joint*/) override
    {
        m_type = JointType::Gear;
    }
    
    /// @brief Visits a WheelJoint.
    void Visit(const WheelJoint& /*joint*/) override
    {
        m_type = JointType::Wheel;
    }
    
    /// @brief Visits a WeldJoint.
    void Visit(const WeldJoint& /*joint*/) override
    {
        m_type = JointType::Weld;
    }
    
    /// @brief Visits a FrictionJoint.
    void Visit(const FrictionJoint& /*joint*/) override
    {
        m_type = JointType::Friction;
    }
    
    /// @brief Visits a RopeJoint.
    void Visit(const RopeJoint& /*joint*/) override
    {
        m_type = JointType::Rope;
    }
    
    /// @brief Visits a MotorJoint.
    void Visit(const MotorJoint& /*joint*/) override
    {
        m_type = JointType::Motor;
    }
    
    /// @brief Gets the type of joint that had been visited.
    Optional<JointType> GetType() const noexcept
    {
        return m_type;
    }
    
private:
    Optional<JointType> m_type;
};

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_TYPEJOINTVISITOR_HPP
