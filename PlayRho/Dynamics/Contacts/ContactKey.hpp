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

#ifndef ContactKey_hpp
#define ContactKey_hpp

/// @file
/// Declaration of the ContactKey class.

#include <PlayRho/Common/Settings.hpp>
#include <utility>
#include <algorithm>
#include <functional>
#include <cassert>

namespace playrho
{
    class Fixture;
    struct FixtureProxy;
    class Contact;

    /// @brief Key value class for contacts.
    class ContactKey
    {
    public:
        using Index = ContactCounter;
        
        constexpr ContactKey() noexcept
        {
            // Intentionally empty
        }
        
        constexpr ContactKey(Index fp1, Index fp2) noexcept:
            m_ids{std::minmax(fp1, fp2)}
        {
            // Intentionally empty
        }

        constexpr Index GetMin() const noexcept
        {
            return m_ids.first;
        }
        
        constexpr Index GetMax() const noexcept
        {
            return m_ids.second;
        }

    private:
        std::pair<Index, Index> m_ids{static_cast<Index>(-1), static_cast<Index>(-1)};
    };

    constexpr bool operator== (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return lhs.GetMin() == rhs.GetMin() && lhs.GetMax() == rhs.GetMax();
    }
    
    constexpr bool operator!= (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return !(lhs == rhs);
    }

    constexpr bool operator< (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() < rhs.GetMin())
            || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() < rhs.GetMax()));
    }
    
    constexpr bool operator<= (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() < rhs.GetMin())
        || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() <= rhs.GetMax()));
    }
    
    constexpr bool operator> (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() > rhs.GetMin())
            || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() > rhs.GetMax()));
    }
    
    constexpr bool operator>= (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() > rhs.GetMin())
        || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() >= rhs.GetMax()));
    }

    using KeyedContactPtr = std::pair<ContactKey, Contact*>;
    
    ContactKey GetContactKey(const FixtureProxy& fpA, const FixtureProxy& fpB) noexcept;
    
    ContactKey GetContactKey(const Fixture* fixtureA, ChildCounter childIndexA,
                             const Fixture* fixtureB, ChildCounter childIndexB) noexcept;
    
    ContactKey GetContactKey(const Contact& contact) noexcept;

} // namespace playrho

namespace std
{
    /// @brief Hash function object specialization for ContactKey.
    template <>
    struct hash<playrho::ContactKey>
    {
        using argument_type = playrho::ContactKey;
        using result_type = std::size_t;

        constexpr std::size_t operator()(const playrho::ContactKey& key) const
        {
            // Use simple and fast Knuth multiplicative hash...
            const auto a = std::size_t{key.GetMin()} * 2654435761u;
            const auto b = std::size_t{key.GetMax()} * 2654435761u;
            return a ^ b;
        }
    };
}

namespace playrho
{
    inline Contact* GetContactPtr(KeyedContactPtr value)
    {
        return value.second;
    }
}

#endif /* ContactKey_hpp */
