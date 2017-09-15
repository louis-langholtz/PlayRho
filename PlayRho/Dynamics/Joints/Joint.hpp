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

#ifndef PLAYRHO_JOINT_HPP
#define PLAYRHO_JOINT_HPP

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Dynamics/Joints/JointDef.hpp>

#include <unordered_map>
#include <vector>
#include <utility>
#include <stdexcept>

namespace playrho {

class Body;
class Joint;
class StepConf;
struct Velocity;
struct ConstraintSolverConf;
class BodyConstraint;

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
/// @sa JointFreeFunctions
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

    /// @brief Gets the type of the concrete joint.
    JointType GetType() const noexcept;

    /// @brief Gets the first body attached to this joint.
    Body* GetBodyA() const noexcept;

    /// @brief Gets the second body attached to this joint.
    Body* GetBodyB() const noexcept;

    /// Get the anchor point on bodyA in world coordinates.
    virtual Length2D GetAnchorA() const = 0;

    /// Get the anchor point on bodyB in world coordinates.
    virtual Length2D GetAnchorB() const = 0;

    /// Get the linear reaction on bodyB at the joint anchor.
    virtual Momentum2D GetLinearReaction() const = 0;

    /// Get the angular reaction on bodyB.
    virtual AngularMomentum GetAngularReaction() const = 0;

    /// Get the user data pointer.
    void* GetUserData() const noexcept;

    /// Set the user data pointer.
    void SetUserData(void* data) noexcept;

    /// @brief Gets collide connected.
    /// @note Modifying the collide connect flag won't work correctly because
    ///   the flag is only checked when fixture AABBs begin to overlap.
    bool GetCollideConnected() const noexcept;

    /// @brief Shifts the origin for any points stored in world coordinates.
    virtual void ShiftOrigin(const Length2D newOrigin) { NOT_USED(newOrigin);  }

    virtual ~Joint() = default;

protected:
    
    /// @brief Initializing constructor.
    Joint(const JointDef& def);

private:
    friend class JointAtty;

    /// Flags type data type.
    using FlagsType = std::uint8_t;

    /// @brief Flags stored in m_flags
    enum Flag: FlagsType
    {
        // Used when crawling contact graph when forming islands.
        e_islandFlag = 0x01,

        e_collideConnectedFlag = 0x02
    };

    static constexpr FlagsType GetFlags(const JointDef& def) noexcept;

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

    /// Destroys the given joint.
    /// @note This calls the joint's destructor.
    static void Destroy(const Joint* joint);

    /// Initializes velocity constraint data based on the given solver data.
    /// @note This MUST be called prior to calling <code>SolveVelocityConstraints</code>.
    /// @sa SolveVelocityConstraints.
    virtual void InitVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step,
                                         const ConstraintSolverConf& conf) = 0;

    /// Solves velocity constraints for the given solver data.
    /// @pre <code>InitVelocityConstraints</code> has been called.
    /// @sa InitVelocityConstraints.
    /// @return <code>true</code> if velocity is "solved", <code>false</code> otherwise.
    virtual bool SolveVelocityConstraints(BodyConstraintsMap& bodies, const StepConf& step) = 0;

    // This returns true if the position errors are within tolerance.
    virtual bool SolvePositionConstraints(BodyConstraintsMap& bodies,
                                          const ConstraintSolverConf& conf) const = 0;

    bool IsIslanded() const noexcept;
    void SetIslanded() noexcept;
    void UnsetIslanded() noexcept;

    Body* const m_bodyA;
    Body* const m_bodyB;
    void* m_userData;
    const JointType m_type;
    FlagsType m_flags = 0; ///< Flags. 1-byte.
};

constexpr inline Joint::FlagsType Joint::GetFlags(const JointDef& def) noexcept
{
    auto flags = Joint::FlagsType(0);
    if (def.collideConnected)
    {
        flags |= e_collideConnectedFlag;
    }
    return flags;
}

inline Joint::Joint(const JointDef& def):
    m_type{def.type}, m_bodyA{def.bodyA}, m_bodyB{def.bodyB},
    m_flags{GetFlags(def)}, m_userData{def.userData}
{
    // Intentionally empty.
}

inline JointType Joint::GetType() const noexcept
{
    return m_type;
}

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
    return m_flags & e_collideConnectedFlag;
}

inline bool Joint::IsIslanded() const noexcept
{
    return m_flags & e_islandFlag;
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

/// @defgroup JointFreeFunctions Joint free functions.
/// @details A collection of non-member, non-friend functions that operate on Joint objects.
/// @sa Joint.
/// @{

/// @brief Short-cut function to determine if both bodies are enabled.
bool IsEnabled(const Joint& j) noexcept;

/// @brief Wakes up the joined bodies.
void SetAwake(Joint& j) noexcept;

/// @brief Gets the world index of the given joint.
JointCounter GetWorldIndex(const Joint* joint);

/// @}

#ifdef PLAYRHO_PROVIDE_VECTOR_AT
/// @brief Provides referenced access to the identified element of the given container.
BodyConstraintPtr& At(std::vector<BodyConstraintPair>& container, const Body* key);
#endif

/// @brief Provides referenced access to the identified element of the given container.
BodyConstraintPtr& At(std::unordered_map<const Body*, BodyConstraint*>& container,
                      const Body* key);

} // namespace playrho

#endif
