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

#ifndef PLAYRHO_CONTACT_KEY_HPP
#define PLAYRHO_CONTACT_KEY_HPP

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
        
        /// @brief Index type.
        using Index = ContactCounter;
        
        constexpr ContactKey() noexcept
        {
            // Intentionally empty
        }
        
        /// @brief Initializing constructor.
        constexpr ContactKey(Index fp1, Index fp2) noexcept:
            m_ids{std::minmax(fp1, fp2)}
        {
            // Intentionally empty
        }

        /// @brief Gets the minimum index value.
        constexpr Index GetMin() const noexcept
        {
            return m_ids.first;
        }
        
        /// @brief Gets the maximum index value.
        constexpr Index GetMax() const noexcept
        {
            return m_ids.second;
        }

    private:
        std::pair<Index, Index> m_ids{static_cast<Index>(-1), static_cast<Index>(-1)};
    };

    /// @brief Equality operator.
    constexpr bool operator== (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return lhs.GetMin() == rhs.GetMin() && lhs.GetMax() == rhs.GetMax();
    }
    
    /// @brief Inequality operator.
    constexpr bool operator!= (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return !(lhs == rhs);
    }

    /// @brief Less-than operator.
    constexpr bool operator< (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() < rhs.GetMin())
            || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() < rhs.GetMax()));
    }
    
    /// @brief Less-than or equal-to operator.
    constexpr bool operator<= (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() < rhs.GetMin())
        || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() <= rhs.GetMax()));
    }
    
    /// @brief Greater-than operator.
    constexpr bool operator> (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() > rhs.GetMin())
            || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() > rhs.GetMax()));
    }
    
    /// @brief Greater-than or equal-to operator.
    constexpr bool operator>= (const ContactKey lhs, const ContactKey rhs) noexcept
    {
        return (lhs.GetMin() > rhs.GetMin())
        || ((lhs.GetMin() == rhs.GetMin()) && (lhs.GetMax() >= rhs.GetMax()));
    }

    /// @brief Keyed contact pointer.
    using KeyedContactPtr = std::pair<ContactKey, Contact*>;
    
    /// @brief Gets the ContactKey for the given parameters.
    ContactKey GetContactKey(const FixtureProxy& fpA, const FixtureProxy& fpB) noexcept;
    
    /// @brief Gets the ContactKey for the given parameters.
    ContactKey GetContactKey(const Fixture* fixtureA, ChildCounter childIndexA,
                             const Fixture* fixtureB, ChildCounter childIndexB) noexcept;
    
    /// @brief Gets the ContactKey for the given contact.
    ContactKey GetContactKey(const Contact& contact) noexcept;

    /// @brief Gets the contact pointer for the given value.
    inline Contact* GetContactPtr(KeyedContactPtr value)
    {
        return value.second;
    }

} // namespace playrho

namespace std
{
    /// @brief Hash function object specialization for ContactKey.
    template <>
    struct hash<playrho::ContactKey>
    {
        /// @brief Argument type.
        using argument_type = playrho::ContactKey;
        
        /// @brief Result type.
        using result_type = std::size_t;

        /// @brief Function object operator.
        constexpr std::size_t operator()(const playrho::ContactKey& key) const
        {
            // Use simple and fast Knuth multiplicative hash...
            const auto a = std::size_t{key.GetMin()} * 2654435761u;
            const auto b = std::size_t{key.GetMax()} * 2654435761u;
            return a ^ b;
        }
    };
}

#endif /* PLAYRHO_CONTACT_KEY_HPP */
