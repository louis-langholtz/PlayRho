/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef ContactKey_hpp
#define ContactKey_hpp

/// @file
/// Declaration of the ContactKey class.

#include <Box2D/Common/Settings.hpp>
#include <utility>
#include <functional>
#include <cassert>

namespace box2d
{
    class Fixture;
    struct FixtureProxy;
    class Contact;

    /// @brief Key value class for contacts.
    class ContactKey
    {
    public:
        static ContactKey Get(contact_count_t a, contact_count_t b) noexcept;
        
        static constexpr std::size_t Hash(const ContactKey key) noexcept
        {
            // Use simple and fast Knuth multiplicative hash...
            const auto a = std::size_t{key.m_fp1} * 2654435761u;
            const auto b = std::size_t{key.m_fp2} * 2654435761u;
            return a ^ b;
        }

        static constexpr int Compare(const ContactKey lhs, const ContactKey rhs)
        {
            if (lhs.m_fp1 < rhs.m_fp1)
            {
                return -1;
            }
            if (lhs.m_fp1 > rhs.m_fp1)
            {
                return +1;
            }

            // From here on lhs.m_fp1 == rhs.m_fp1
            if (lhs.m_fp2 < rhs.m_fp2)
            {
                return -1;
            }
            if (lhs.m_fp2 > rhs.m_fp2)
            {
                return +1;
            }

            return 0;
        }
        
    private:
        constexpr ContactKey(contact_count_t fp1, contact_count_t fp2) noexcept:
        	m_fp1{fp1}, m_fp2{fp2}
        {
            assert(fp1 <= fp2);
        }
    
        contact_count_t m_fp1; ///< Fixture proxy 1 (ID).
        contact_count_t m_fp2; ///< Fixture proxy 2 (ID).
    };
    
    
    ContactKey GetContactKey(const FixtureProxy& fpA, const FixtureProxy& fpB) noexcept;
    
    ContactKey GetContactKey(const Fixture* fixtureA, child_count_t childIndexA,
                             const Fixture* fixtureB, child_count_t childIndexB) noexcept;

    ContactKey GetContactKey(const Contact& contact) noexcept;

    constexpr bool operator== (const box2d::ContactKey lhs,
                               const box2d::ContactKey rhs) noexcept
    {
        return ContactKey::Compare(lhs, rhs) == 0;
    }
    
    constexpr bool operator!= (const box2d::ContactKey lhs,
                               const box2d::ContactKey rhs) noexcept
    {
        return !(lhs == rhs);
    }

    using KeyedContactPtr = std::pair<ContactKey, Contact*>;

} // namespace box2d

namespace std
{
    template <>
    struct less<box2d::ContactKey>
    {
        constexpr bool operator()(const box2d::ContactKey& lhs,
                                  const box2d::ContactKey& rhs) const
        {
            return box2d::ContactKey::Compare(lhs, rhs) < 0;
        }
    };
    
    template <>
    struct hash<box2d::ContactKey>
    {
        using argument_type = box2d::ContactKey;
        using result_type = std::size_t;

        constexpr std::size_t operator()(const box2d::ContactKey& key) const
        {
            return box2d::ContactKey::Hash(key);
        }
    };
    
    template <>
    struct equal_to<box2d::ContactKey>
    {
        constexpr bool operator()( const box2d::ContactKey& lhs, const box2d::ContactKey& rhs ) const
        {
            return box2d::ContactKey::Compare(lhs, rhs) == 0;
        }
    };
}

namespace box2d
{
    inline Contact* GetContactPtr(KeyedContactPtr value)
    {
        return value.second;
    }
}

#endif /* ContactKey_hpp */
