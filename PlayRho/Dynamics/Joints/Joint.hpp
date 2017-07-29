/*
* Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_JOINT_H
#define B2_JOINT_H

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Common/Span.hpp>
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

using BodyConstraintPtr = BodyConstraint*;
using BodyConstraintPair = std::pair<const Body*, BodyConstraintPtr>;

// #define USE_VECTOR_MAP
#ifdef USE_VECTOR_MAP
using BodyConstraintsMap = std::vector<std::pair<const Body*, BodyConstraintPtr>>;
#else
using BodyConstraintsMap = std::unordered_map<const Body*, BodyConstraint*>;
#endif

enum class JointType: std::uint8_t
{
    Unknown,
    Revolute,
    Prismatic,
    Distance,
    Pulley,
    Mouse,
    Gear,
    Wheel,
    Weld,
    Friction,
    Rope,
    Motor
};

/// @brief Abstract base Joint definition class.
/// @details Joint definitions are used to construct joints.
/// @note This class is not meant to be directly instantiated; it is meant
///   to be inherreted from.
struct JointDef
{
    /// Deleted default constructor for abstract base class.
    JointDef() = delete; // deleted to prevent direct instantiation.

    constexpr JointDef(JointType t) noexcept:
        type{t}
    {
        // Intentionally empty.        
    }

    constexpr JointDef(JointType t, Body* bA, Body* bB, bool cc = false,
                       void* u = nullptr) noexcept:
        type{t}, bodyA{bA}, bodyB{bB}, collideConnected{cc}, userData{u}
    {
        // Intentionally empty.
    }

    constexpr JointDef& UseBodyA(Body* body) noexcept { bodyA = body; return *this; }
    constexpr JointDef& UseBodyB(Body* body) noexcept { bodyB = body; return *this; }

    /// The joint type is set automatically for concrete joint types.
    const JointType type;

    /// @brief First attached body.
    Body* bodyA = nullptr;

    /// @brief Second attached body.
    Body* bodyB = nullptr;

    /// Set this flag to true if the attached bodies should collide.
    bool collideConnected = false;
    
    /// Use this to attach application specific data to your joints.
    void* userData = nullptr;
};

/// @brief Base joint class.
/// @details Joints are used to constraint two bodies together in various fashions.
///   Some joints also feature limits and motors.
class Joint
{
public:
    enum LimitState
    {
        e_inactiveLimit,
        e_atLowerLimit,
        e_atUpperLimit,
        e_equalLimits
    };
    
    static bool IsOkay(const JointDef& def) noexcept;

    /// @brief Gets the type of the concrete joint.
    constexpr JointType GetType() const noexcept;

    /// @brief Gets the first body attached to this joint.
    constexpr Body* GetBodyA() const noexcept;

    /// @brief Gets the second body attached to this joint.
    constexpr Body* GetBodyB() const noexcept;

    /// Get the anchor point on bodyA in world coordinates.
    virtual Length2D GetAnchorA() const = 0;

    /// Get the anchor point on bodyB in world coordinates.
    virtual Length2D GetAnchorB() const = 0;

    /// Get the reaction force on bodyB at the joint anchor in Newtons.
    virtual Force2D GetReactionForce(Frequency inv_dt) const = 0;

    /// Get the reaction torque on bodyB in N*m.
    virtual Torque GetReactionTorque(Frequency inv_dt) const = 0;

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
    constexpr Joint(const JointDef& def);
    
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
    virtual bool SolvePositionConstraints(BodyConstraintsMap& bodies, const ConstraintSolverConf& conf) const = 0;
    
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

constexpr inline Joint::Joint(const JointDef& def):
    m_type{def.type}, m_bodyA{def.bodyA}, m_bodyB{def.bodyB},
    m_flags{GetFlags(def)}, m_userData{def.userData}
{
    // Intentionally empty.
}

constexpr inline JointType Joint::GetType() const noexcept
{
    return m_type;
}

constexpr inline Body* Joint::GetBodyA() const noexcept
{
    return m_bodyA;
}

constexpr inline Body* Joint::GetBodyB() const noexcept
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
    
/// @brief Short-cut function to determine if both bodies are enabled.
bool IsEnabled(const Joint& j) noexcept;

// @brief Wakes up the joined bodies.
void SetAwake(Joint& j) noexcept;

JointCounter GetWorldIndex(const Joint* joint);

void Set(JointDef& def, const Joint& joint) noexcept;

inline BodyConstraintPtr& At(std::vector<BodyConstraintPair>& container, const Body* key)
{
    auto last = std::end(container);
    auto first = std::begin(container);
    first = std::lower_bound(first, last, key, [](const BodyConstraintPair &a, const Body* b){
        return a.first < b;
    });
    if (first == last || key != (*first).first)
    {
        throw std::out_of_range{"invalid key"};
    }
    return (*first).second;
}

inline BodyConstraintPtr& At(std::unordered_map<const Body*, BodyConstraint*>& container,
                             const Body* key)
{
    return container.at(key);
}

} // namespace playrho

#endif
