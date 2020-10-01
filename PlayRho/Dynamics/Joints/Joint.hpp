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

#ifndef PLAYRHO_DYNAMICS_JOINTS_JOINT_HPP
#define PLAYRHO_DYNAMICS_JOINTS_JOINT_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Dynamics/BodyID.hpp>

#include <vector>
#include <utility>
#include <stdexcept>

namespace playrho {
class StepConf;
struct ConstraintSolverConf;

namespace d2 {

class Body;
struct Velocity;
class BodyConstraint;
class JointVisitor;
struct JointConf;
class World;

/// @defgroup JointsGroup Joint Classes
/// @brief The user creatable classes that specify constraints on one or more Body instances.
/// @ingroup ConstraintsGroup

/// @brief Base joint class.
///
/// @details Joints are constraints that are used to constrain one or more bodies in various
///   fashions. Some joints also feature limits and motors.
///
/// @ingroup JointsGroup
/// @ingroup PhysicalEntities
///
/// @see World
///
class Joint
{
public:
    using BodyConstraintsMap = std::vector<BodyConstraint>;
    
    /// @brief Limit state.
    /// @note Only used by joints that implement some notion of a limited range.
    enum LimitState
    {
        /// @brief Inactive limit.
        e_inactiveLimit,

        /// @brief At-lower limit.
        e_atLowerLimit,
        
        /// @brief At-upper limit.
        e_atUpperLimit,
        
        /// @brief Equal limit.
        /// @details Equal limit is used to indicate that a joint's upper and lower limits
        ///   are approximately the same.
        e_equalLimits
    };

    /// @brief Is the given definition okay.
    static bool IsOkay(const JointConf& def) noexcept;

    /// @brief Dynamically allocates and instantiates the out-type from the given data.
    template <class OUT_TYPE, class IN_TYPE>
    static OUT_TYPE* Create(IN_TYPE def)
    {
        if (OUT_TYPE::IsOkay(def))
        {
            return new OUT_TYPE(def);
        }
        throw InvalidArgument("definition not okay");
    }

    /// @brief Creates a new joint based on the given definition.
    /// @throws InvalidArgument if given a joint definition with a type that's not recognized.
    static Joint* Create(const JointConf& def);

    /// @brief Destroys the given joint.
    /// @note This calls the joint's destructor.
    static void Destroy(const Joint* joint) noexcept;

    /// @brief Initializing constructor.
    explicit Joint(const JointConf& def);

    virtual ~Joint() noexcept = default;

    /// @brief Gets the first body attached to this joint.
    BodyID GetBodyA() const noexcept;

    /// @brief Gets the second body attached to this joint.
    BodyID GetBodyB() const noexcept;

    /// Get the anchor point on body-A in local coordinates.
    virtual Length2 GetLocalAnchorA() const noexcept = 0;

    /// Get the anchor point on body-B in local coordinates.
    virtual Length2 GetLocalAnchorB() const noexcept = 0;

    /// Get the linear reaction on body-B at the joint anchor.
    virtual Momentum2 GetLinearReaction() const = 0;

    /// Get the angular reaction on body-B.
    virtual AngularMomentum GetAngularReaction() const = 0;
    
    /// @brief Accepts a visitor.
    /// @details This is the Accept method definition of a "visitor design pattern" for
    ///   for doing joint subclass specific types of processing for a constant joint.
    /// @see JointVisitor
    /// @see https://en.wikipedia.org/wiki/Visitor_pattern
    virtual void Accept(JointVisitor& visitor) const = 0;
    
    /// @brief Accepts a visitor.
    /// @details This is the Accept method definition of a "visitor design pattern" for
    ///   for doing joint subclass specific types of processing.
    /// @see JointVisitor
    /// @see https://en.wikipedia.org/wiki/Visitor_pattern
    virtual void Accept(JointVisitor& visitor) = 0;

    /// Get the user data pointer.
    void* GetUserData() const noexcept;

    /// Set the user data pointer.
    void SetUserData(void* data) noexcept;

    /// @brief Gets collide connected.
    /// @note Modifying the collide connect flag won't work correctly because
    ///   the flag is only checked when fixture AABBs begin to overlap.
    bool GetCollideConnected() const noexcept;

    /// @brief Shifts the origin for any points stored in world coordinates.
    /// @return <code>true</code> if shift done, <code>false</code> otherwise.
    virtual bool ShiftOrigin(Length2) { return false;  }

    /// @brief Initializes velocity constraint data based on the given solver data.
    /// @note This MUST be called prior to calling <code>SolveVelocityConstraints</code>.
    /// @see SolveVelocityConstraints.
    virtual void InitVelocityConstraints(BodyConstraintsMap& bodies,
                                         const playrho::StepConf& step,
                                         const ConstraintSolverConf& conf) = 0;

    /// @brief Solves velocity constraint.
    /// @pre <code>InitVelocityConstraints</code> has been called.
    /// @see InitVelocityConstraints.
    /// @return <code>true</code> if velocity is "solved", <code>false</code> otherwise.
    virtual bool SolveVelocityConstraints(BodyConstraintsMap& bodies,
                                          const playrho::StepConf& step) = 0;

    /// @brief Solves the position constraint.
    /// @return <code>true</code> if the position errors are within tolerance.
    virtual bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                          const ConstraintSolverConf& conf) const = 0;

    /// @brief Whether this joint is in the is-in-island state.
    bool IsIslanded() const noexcept;
    
    /// @brief Sets this joint to be in the is-in-island state.
    void SetIslanded() noexcept;
    
    /// @brief Unsets this joint from being in the is-in-island state.
    void UnsetIslanded() noexcept;

private:
    /// Flags type data type.
    using FlagsType = std::uint8_t;

    /// @brief Flags stored in m_flags
    enum Flag: FlagsType
    {
        // Used when crawling contact graph when forming islands.
        e_islandFlag = 0x01u,
        e_collideConnectedFlag = 0x02u
    };

    /// @brief Gets the flags value for the given joint definition.
    static FlagsType GetFlags(const JointConf& def) noexcept;

    void* m_userData; ///< User data.
    BodyID const m_bodyA; ///< Body A.
    BodyID const m_bodyB; ///< Body B.
    FlagsType m_flags = 0u; ///< Flags. 1-byte.
};

inline BodyID Joint::GetBodyA() const noexcept
{
    return m_bodyA;
}

inline BodyID Joint::GetBodyB() const noexcept
{
    return m_bodyB;
}

inline void* Joint::GetUserData() const noexcept
{
    return m_userData;
}

inline void Joint::SetUserData(void* data) noexcept
{
    m_userData = data;
}

inline bool Joint::GetCollideConnected() const noexcept
{
    return (m_flags & e_collideConnectedFlag) != 0u;
}

inline bool Joint::IsIslanded() const noexcept
{
    return (m_flags & e_islandFlag) != 0u;
}

inline void Joint::SetIslanded() noexcept
{
    m_flags |= e_islandFlag;
}

inline void Joint::UnsetIslanded() noexcept
{
    m_flags &= ~e_islandFlag;
}

// Free functions...

/// @brief Provides referenced access to the identified element of the given container.
BodyConstraint& At(std::vector<BodyConstraint>& container, BodyID key);

/// @brief Provides a human readable C-style string uniquely identifying the given limit state.
const char* ToString(Joint::LimitState val) noexcept;

/// @brief Increment motor speed.
/// @details Template function for incrementally changing the motor speed of a joint that has
///   the <code>SetMotorSpeed</code> and <code>GetMotorSpeed</code> methods.
template <class T>
inline void IncMotorSpeed(T& j, AngularVelocity delta)
{
    j.SetMotorSpeed(j.GetMotorSpeed() + delta);
}

Angle GetReferenceAngle(const Joint& object);

UnitVec GetLocalAxisA(const Joint& object);

} // namespace d2
} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_JOINT_HPP
