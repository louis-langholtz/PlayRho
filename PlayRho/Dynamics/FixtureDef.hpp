/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef FixtureDef_hpp
#define FixtureDef_hpp

/// @file
/// Declarations of the FixtureDef struct and any free functions associated with it.

#include <PlayRho/Dynamics/Filter.hpp>

namespace box2d {
    
    class Fixture;

    /// @brief Fixture definition.
    ///
    /// @details A fixture definition is used to create a fixture.
    /// @sa Body::CreateFixture.
    ///
    struct FixtureDef
    {
        constexpr FixtureDef& UseUserData(void* value) noexcept;
        constexpr FixtureDef& UseIsSensor(bool value) noexcept;
        constexpr FixtureDef& UseFilter(Filter value) noexcept;
        
        /// Use this to store application specific fixture data.
        void* userData = nullptr;
        
        /// A sensor shape collects contact information but never generates a collision
        /// response.
        bool isSensor = false;
        
        /// Contact filtering data.
        Filter filter;
    };
    
    constexpr inline FixtureDef& FixtureDef::UseUserData(void* value) noexcept
    {
        userData = value;
        return *this;
    }
    
    constexpr inline FixtureDef& FixtureDef::UseIsSensor(bool value) noexcept
    {
        isSensor = value;
        return *this;
    }
    
    constexpr inline FixtureDef& FixtureDef::UseFilter(Filter value) noexcept
    {
        filter = value;
        return *this;
    }

    constexpr FixtureDef GetDefaultFixtureDef() noexcept
    {
        return FixtureDef{};
    }
    
    FixtureDef GetFixtureDef(const Fixture& fixture) noexcept;

} // namespace box2d

#endif /* FixtureDef_hpp */
