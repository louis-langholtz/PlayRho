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

#include <unordered_map>
#include <vector>
#include <utility>
#include <stdexcept>

namespace playrho {

class Body;
class StepConf;
struct Velocity;
struct ConstraintSolverConf;
class BodyConstraint;
class JointVisitor;
struct JointDef;

/// @defgroup JointsGroup Joint Classes
/// @details These are user creatable classes that specify constraints on one or more
///   Body instances.

/// @brief A body constraint pointer alias.
using BodyConstraintPtr = BodyConstraint*;

/// @brief A body pointer and body constraint pointer pair alias.
using BodyConstraintPair = std::pair<const Body*, BodyConstraintPtr>;

// #define USE_VECTOR_MAP

/// @brief A body constraints map alias.
using BodyConstraintsMap =
#ifdef USE_VECTOR_MAP
    std::vector<std::pair<const Body*, BodyConstraintPtr>>;
#else
    std::unordered_map<const Body*, BodyConstraint*>;
#endif

/// @brief Base joint class.
///
/// @details Joints are used to constraint two bodies together in various fashions.
///   Some joints also feature limits and motors.
///
/// @ingroup JointsGroup
///
/// @sa World
///
class Joint
{
public:
    
    /// @brief Limit state.
    enum LimitState
    {
        e_inactiveLimit,
        e_atLowerLimit,
        e_atUpperLimit,
        e_equalLimits
    };

    /// @brief Is the given definition okay.
    static bool IsOkay(const JointDef& def) noexcept;

    virtual ~Joint() = default;

    /// @brief Gets the first body attached to this joint.
    Body* GetBodyA() const noexcept;

    /// @brief Gets the second body attached to this joint.
    Body* GetBodyB() const noexcept;

    /// Get the anchor point on bodyA in world coordinates.
    virtual Length2 GetAnchorA() const = 0;

    /// Get the anchor point on bodyB in world coordinates.
    virtual Length2 GetAnchorB() const = 0;

    /// Get the linear reaction on bodyB at the joint anchor.
    virtual Momentum2 GetLinearReaction() const = 0;

    /// Get the angular reaction on bodyB.
    virtual AngularMomentum GetAngularReaction() const = 0;
    
    /// @brief Accepts a visitor.
    virtual void Accept(JointVisitor& visitor) const = 0;

    /// Get the user data pointer.
    void* GetUserData() const noexcept;

    /// Set the user data pointer.
    void SetUserData(void* data) noexcept;

    /// @brief Gets collide connected.
    /// @note Modifying the collide connect flag won't work correctly because
    ///   the flag is only checked when fixture AABBs begin to overlap.
    bool GetCollideConnected() const noexcept;

    /// @brief Shifts the origin for any points stored in world coordinates.
    virtual void ShiftOrigin(const Length2 newOrigin) { NOT_USED(newOrigin);  }

protected:
    
    /// @brief Initializing constructor.
    explicit Joint(const JointDef& def);

private:
    friend class JointAtty;

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
    static FlagsType GetFlags(const JointDef& def) noexcept;

    /// @brief Dynamically allocates and instantiates the out-type from the given data.
    template <class OUT_TYPE, class IN_TYPE>
    static OUT_TYPE* Create(IN_TYPE def)
    {
        if (OUT_TYPE::IsOkay(def))
        {
            return new OUT_TYPE(def);
        }
        return nullptr;
    }
    
    /// @brief Creates a new joint based on the given definition.
    /// @throws InvalidArgument if given a joint definition with a type that's not recognized.
    static Joint* Create(const JointDef& def);

    /// @brief Destroys the given joint.
    /// @note This calls the joint's destructor.
    static void Destroy(const Joint* joint);

    /// @brief Initializes velocity constraint data based on the given solver data.
    /// @note This MUST be called prior to calling <code>SolveVelocityConstraints</code>.
    /// @sa SolveVelocityConstraints.
    virtual void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                         const ConstraintSolverConf& conf) = 0;

    /// @brief Solves velocity constraint.
    /// @pre <code>InitVelocityConstraints</code> has been called.
    /// @sa InitVelocityConstraints.
    /// @return <code>true</code> if velocity is "solved", <code>false</code> otherwise.
    virtual bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) = 0;

    /// @brief Solves the position constraint.
    /// @return <code>true</code> if the position errors are within tolerance.
    virtual bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                          const ConstraintSolverConf& conf) const = 0;

    /// @brief Whether this joint is in the is-islanded state.
    bool IsIslanded() const noexcept;
    
    /// @brief Sets this joint to be in the is-islanded state.
    void SetIslanded() noexcept;
    
    /// @brief Unsets this joint from being in the is-islanded state.
    void UnsetIslanded() noexcept;

    Body* const m_bodyA; ///< Body A.
    Body* const m_bodyB; ///< Body B.
    void* m_userData; ///< User data.
    FlagsType m_flags = 0u; ///< Flags. 1-byte.
};

inline Body* Joint::GetBodyA() const noexcept
{
    return m_bodyA;
}

inline Body* Joint::GetBodyB() const noexcept
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

/// @brief Short-cut function to determine if both bodies are enabled.
/// @relatedalso Joint
bool IsEnabled(const Joint& j) noexcept;

/// @brief Wakes up the joined bodies.
/// @relatedalso Joint
void SetAwake(Joint& j) noexcept;

/// @brief Gets the world index of the given joint.
/// @relatedalso Joint
JointCounter GetWorldIndex(const Joint* joint);

#ifdef PLAYRHO_PROVIDE_VECTOR_AT
/// @brief Provides referenced access to the identified element of the given container.
BodyConstraintPtr& At(std::vector<BodyConstraintPair>& container, const Body* key);
#endif

/// @brief Provides referenced access to the identified element of the given container.
BodyConstraintPtr& At(std::unordered_map<const Body*, BodyConstraint*>& container,
                      const Body* key);

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_JOINTS_JOINT_HPP
