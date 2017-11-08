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

#ifndef PLAYRHO_DYNAMICS_JOINTS_JOINTVISITOR_HPP
#define PLAYRHO_DYNAMICS_JOINTS_JOINTVISITOR_HPP

namespace playrho {

class RevoluteJoint;
class PrismaticJoint;
class DistanceJoint;
class PulleyJoint;
class MouseJoint;
class GearJoint;
class WheelJoint;
class WeldJoint;
class FrictionJoint;
class RopeJoint;
class MotorJoint;

/// @brief Vistor interface for Joint instances.
///
/// @details Interface to inherit from for objects wishing to "visit" joints.
///   This uses the visitor design pattern.
/// @sa https://en.wikipedia.org/wiki/Visitor_pattern .
///
class JointVisitor
{
public:
    virtual ~JointVisitor() = default;
    
    /// @brief Visits a RevoluteJoint.
    virtual void Visit(const RevoluteJoint& joint) = 0;
    
    /// @brief Visits a RevoluteJoint.
    virtual void Visit(RevoluteJoint& joint) = 0;
    
    /// @brief Visits a PrismaticJoint.
    virtual void Visit(const PrismaticJoint& joint) = 0;
    
    /// @brief Visits a PrismaticJoint.
    virtual void Visit(PrismaticJoint& joint) = 0;
    
    /// @brief Visits a DistanceJoint.
    virtual void Visit(const DistanceJoint& joint) = 0;
    
    /// @brief Visits a DistanceJoint.
    virtual void Visit(DistanceJoint& joint) = 0;
    
    /// @brief Visits a PulleyJoint.
    virtual void Visit(const PulleyJoint& joint) = 0;
    
    /// @brief Visits a PulleyJoint.
    virtual void Visit(PulleyJoint& joint) = 0;

    /// @brief Visits a MouseJoint.
    virtual void Visit(const MouseJoint& joint) = 0;

    /// @brief Visits a MouseJoint.
    virtual void Visit(MouseJoint& joint) = 0;

    /// @brief Visits a GearJoint.
    virtual void Visit(const GearJoint& joint) = 0;

    /// @brief Visits a GearJoint.
    virtual void Visit(GearJoint& joint) = 0;

    /// @brief Visits a WheelJoint.
    virtual void Visit(const WheelJoint& joint) = 0;

    /// @brief Visits a WheelJoint.
    virtual void Visit(WheelJoint& joint) = 0;
    
    /// @brief Visits a WeldJoint.
    virtual void Visit(const WeldJoint& joint) = 0;
    
    /// @brief Visits a WeldJoint.
    virtual void Visit(WeldJoint& joint) = 0;
    
    /// @brief Visits a FrictionJoint.
    virtual void Visit(const FrictionJoint& joint) = 0;
    
    /// @brief Visits a FrictionJoint.
    virtual void Visit(FrictionJoint& joint) = 0;
    
    /// @brief Visits a RopeJoint.
    virtual void Visit(const RopeJoint& joint) = 0;
    
    /// @brief Visits a RopeJoint.
    virtual void Visit(RopeJoint& joint) = 0;

    /// @brief Visits a MotorJoint.
    virtual void Visit(const MotorJoint& joint) = 0;

    /// @brief Visits a MotorJoint.
    virtual void Visit(MotorJoint& joint) = 0;
    
protected:
    JointVisitor() = default;
    
    /// @brief Copy constructor.
    JointVisitor(const JointVisitor& other) = default;
    
    /// @brief Move constructor.
    JointVisitor(JointVisitor&& other) = default;
    
    /// @brief Copy assignment operator.
    JointVisitor& operator= (const JointVisitor& other) = default;
    
    /// @brief Move assignment operator.
    JointVisitor& operator= (JointVisitor&& other) = default;
};

/// @brief Constant joint visitor interface class.
class ConstJointVisitor: public JointVisitor
{
public:
    void Visit(const RevoluteJoint& j) override = 0;
    void Visit(RevoluteJoint& j) override { Visit(static_cast<const RevoluteJoint&>(j)); }
    void Visit(const PrismaticJoint& j) override = 0;
    void Visit(PrismaticJoint& j) override { Visit(static_cast<const PrismaticJoint&>(j)); }
    void Visit(const DistanceJoint& j) override = 0;
    void Visit(DistanceJoint& j) override { Visit(static_cast<const DistanceJoint&>(j)); }
    void Visit(const PulleyJoint& j) override = 0;
    void Visit(PulleyJoint& j) override { Visit(static_cast<const PulleyJoint&>(j)); }
    void Visit(const MouseJoint& j) override = 0;
    void Visit(MouseJoint& j) override { Visit(static_cast<const MouseJoint&>(j)); }
    void Visit(const GearJoint& j) override = 0;
    void Visit(GearJoint& j) override { Visit(static_cast<const GearJoint&>(j)); }
    void Visit(const WheelJoint& j) override = 0;
    void Visit(WheelJoint& j) override { Visit(static_cast<const WheelJoint&>(j)); }
    void Visit(const WeldJoint& j) override = 0;
    void Visit(WeldJoint& j) override { Visit(static_cast<const WeldJoint&>(j)); }
    void Visit(const FrictionJoint& j) override = 0;
    void Visit(FrictionJoint& j) override { Visit(static_cast<const FrictionJoint&>(j)); }
    void Visit(const RopeJoint& j) override = 0;
    void Visit(RopeJoint& j) override { Visit(static_cast<const RopeJoint&>(j)); }
    void Visit(const MotorJoint& j) override = 0;
    void Visit(MotorJoint& j) override { Visit(static_cast<const MotorJoint&>(j)); }
};

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_JOINTVISITOR_HPP
