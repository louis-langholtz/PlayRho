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

#ifndef JointDef_hpp
#define JointDef_hpp

#include <cstdint>

namespace playrho {

class Body;
class Joint;

enum class JointType : std::uint8_t
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
    
    constexpr JointDef(JointType t) noexcept : type{t}
    {
        // Intentionally empty.
    }
    
    /// The joint type is set automatically for concrete joint types.
    JointType type;
    
    /// @brief First attached body.
    Body* bodyA = nullptr;
    
    /// @brief Second attached body.
    Body* bodyB = nullptr;
    
    /// Set this flag to true if the attached bodies should collide.
    bool collideConnected = false;
    
    /// Use this to attach application specific data to your joints.
    void* userData = nullptr;
};

template <class T>
struct JointBuilder : JointDef
{
    using value_type = T;
    using reference = value_type&;
    
    constexpr JointBuilder(JointType t) noexcept : JointDef{t}
    {
        // Intentionally empty.
    }
    
    constexpr reference UseBodyA(Body* b) noexcept
    {
        bodyA = b;
        return static_cast<reference>(*this);
    }
    
    constexpr reference UseBodyB(Body* b) noexcept
    {
        bodyB = b;
        return static_cast<reference>(*this);
    }
    
    constexpr reference UseCollideConnected(bool v) noexcept
    {
        collideConnected = v;
        return static_cast<reference>(*this);
    }
    
    constexpr reference UseUserData(void* v) noexcept
    {
        userData = v;
        return static_cast<reference>(*this);
    }
};

void Set(JointDef& def, const Joint& joint) noexcept;

} // namespace playrho

#endif /* JointDef_hpp */
