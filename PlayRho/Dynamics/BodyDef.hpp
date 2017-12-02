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

#ifndef PLAYRHO_DYNAMICS_BODYDEF_HPP
#define PLAYRHO_DYNAMICS_BODYDEF_HPP

/// @file
/// Declarations of the BodyDef struct and free functions associated with it.

#include <PlayRho/Common/Settings.hpp>
#include <PlayRho/Common/BoundedValue.hpp>
#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Dynamics/BodyType.hpp>

namespace playrho {

    class Body;

    /// @brief Body Definition.
    ///
    /// @details A body definition holds all the data needed to construct a rigid body.
    ///   You can safely re-use body definitions.
    ///
    /// @note This is a value class meant for passing in to the World::CreateBody method.
    ///
    /// @sa World.
    ///
    struct BodyDef
    {
        // Builder-styled methods...

        /// @brief Use the given type.
        PLAYRHO_CONSTEXPR inline BodyDef& UseType(BodyType t) noexcept;

        /// @brief Use the given location.
        PLAYRHO_CONSTEXPR inline BodyDef& UseLocation(Length2 l) noexcept;
        
        /// @brief Use the given angle.
        PLAYRHO_CONSTEXPR inline BodyDef& UseAngle(Angle a) noexcept;
        
        /// @brief Use the given linear velocity.
        PLAYRHO_CONSTEXPR inline BodyDef& UseLinearVelocity(LinearVelocity2 v) noexcept;
        
        /// @brief Use the given angular velocity.
        PLAYRHO_CONSTEXPR inline BodyDef& UseAngularVelocity(AngularVelocity v) noexcept;
        
        /// @brief Use the given linear acceleration.
        PLAYRHO_CONSTEXPR inline BodyDef& UseLinearAcceleration(LinearAcceleration2 v) noexcept;
        
        /// @brief Use the given angular acceleration.
        PLAYRHO_CONSTEXPR inline BodyDef& UseAngularAcceleration(AngularAcceleration v) noexcept;
        
        /// @brief Use the given linear damping.
        PLAYRHO_CONSTEXPR inline BodyDef& UseLinearDamping(NonNegative<Frequency> v) noexcept;
        
        /// @brief Use the given angular damping.
        PLAYRHO_CONSTEXPR inline BodyDef& UseAngularDamping(NonNegative<Frequency> v) noexcept;
        
        /// @brief Use the given under active time.
        PLAYRHO_CONSTEXPR inline BodyDef& UseUnderActiveTime(Time v) noexcept;
        
        /// @brief Use the given allow sleep value.
        PLAYRHO_CONSTEXPR inline BodyDef& UseAllowSleep(bool value) noexcept;
        
        /// @brief Use the given awake value.
        PLAYRHO_CONSTEXPR inline BodyDef& UseAwake(bool value) noexcept;
        
        /// @brief Use the given fixed rotation state.
        PLAYRHO_CONSTEXPR inline BodyDef& UseFixedRotation(bool value) noexcept;
        
        /// @brief Use the given bullet state.
        PLAYRHO_CONSTEXPR inline BodyDef& UseBullet(bool value) noexcept;
        
        /// @brief Use the given enabled state.
        PLAYRHO_CONSTEXPR inline BodyDef& UseEnabled(bool value) noexcept;
        
        /// @brief Use the given user data.
        PLAYRHO_CONSTEXPR inline BodyDef& UseUserData(void* value) noexcept;
        
        // Public member variables...
        
        /// @brief Type of the body: static, kinematic, or dynamic.
        /// @note If a dynamic body would have zero mass, the mass is set to one.
        BodyType type = BodyType::Static;
        
        /// The world location of the body. Avoid creating bodies at the origin
        /// since this can lead to many overlapping shapes.
        Length2 location = Length2{};
        
        /// The world angle of the body.
        Angle angle = 0_deg;
        
        /// The linear velocity of the body's origin in world co-ordinates (in m/s).
        LinearVelocity2 linearVelocity = LinearVelocity2{};
        
        /// The angular velocity of the body.
        AngularVelocity angularVelocity = 0_rpm;
        
        /// Initial linear acceleration of the body.
        /// @note Usually this should be 0.
        LinearAcceleration2 linearAcceleration = LinearAcceleration2{};
        
        /// Initial angular acceleration of the body.
        /// @note Usually this should be 0.
        AngularAcceleration angularAcceleration = AngularAcceleration{0};
        
        /// Linear damping is use to reduce the linear velocity. The damping parameter
        /// can be larger than 1 but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        NonNegative<Frequency> linearDamping = NonNegative<Frequency>{0};
        
        /// Angular damping is use to reduce the angular velocity. The damping parameter
        /// can be larger than 1 but the damping effect becomes sensitive to the
        /// time step when the damping parameter is large.
        NonNegative<Frequency> angularDamping = NonNegative<Frequency>{0};
        
        /// Under-active time.
        /// @details Set this to the value retrieved from Body::GetUnderActiveTime() or leave it as 0.
        Time underActiveTime = 0_s;
        
        /// Set this flag to false if this body should never fall asleep. Note that
        /// this increases CPU usage.
        bool allowSleep = true;
        
        /// Is this body initially awake or sleeping?
        bool awake = true;
        
        /// Should this body be prevented from rotating? Useful for characters.
        bool fixedRotation = false;
        
        /// Is this a fast moving body that should be prevented from tunneling through
        /// other moving bodies? Note that all bodies are prevented from tunneling through
        /// kinematic and static bodies. This setting is only considered on dynamic bodies.
        /// @note Use this flag sparingly since it increases processing time.
        bool bullet = false;
        
        /// Does this body start out enabled?
        bool enabled = true;
        
        /// Use this to store application specific body data.
        void* userData = nullptr;
    };
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseType(BodyType t) noexcept
    {
        type = t;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseLocation(Length2 l) noexcept
    {
        location = l;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseAngle(Angle a) noexcept
    {
        angle = a;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseLinearVelocity(LinearVelocity2 v) noexcept
    {
        linearVelocity = v;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseLinearAcceleration(LinearAcceleration2 v) noexcept
    {
        linearAcceleration = v;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseAngularVelocity(AngularVelocity v) noexcept
    {
        angularVelocity = v;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseAngularAcceleration(AngularAcceleration v) noexcept
    {
        angularAcceleration = v;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseLinearDamping(NonNegative<Frequency> v) noexcept
    {
        linearDamping = v;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseAngularDamping(NonNegative<Frequency> v) noexcept
    {
        angularDamping = v;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseUnderActiveTime(Time v) noexcept
    {
        underActiveTime = v;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseAllowSleep(bool value) noexcept
    {
        allowSleep = value;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseAwake(bool value) noexcept
    {
        awake = value;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseFixedRotation(bool value) noexcept
    {
        fixedRotation = value;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseBullet(bool value) noexcept
    {
        bullet = value;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseEnabled(bool value) noexcept
    {
        enabled = value;
        return *this;
    }
    
    PLAYRHO_CONSTEXPR inline BodyDef& BodyDef::UseUserData(void* value) noexcept
    {
        userData = value;
        return *this;
    }
    
    /// @brief Gets the default body definition.
    /// @relatedalso BodyDef
    PLAYRHO_CONSTEXPR inline BodyDef GetDefaultBodyDef() noexcept
    {
        return BodyDef{};
    }

    /// @brief Gets the body definition for the given body.
    /// @param body Body to get the BodyDef for.
    /// @relatedalso Body
    BodyDef GetBodyDef(const Body& body) noexcept;

} // namespace playrho

#endif // PLAYRHO_DYNAMICS_BODYDEF_HPP
