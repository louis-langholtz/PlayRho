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

#ifndef PLAYRHO_COMMON_VALUERANGE_HPP
#define PLAYRHO_COMMON_VALUERANGE_HPP

#include <PlayRho/Common/BoundedValue.hpp>
#include <algorithm>
#include <limits>
#include <iostream>

namespace playrho {
    
    /// @brief Value range template type.
    /// @details This type encapsulates a min-max value range relationship.
    /// @invariant The min and max values can only be the result of
    ///   <code>std::minmax(a, b)</code> or the special values of the "highest" and
    ///   "lowest" values supported by the type for this class respectively indicating
    ///   the "unset" value.
    /// @sa https://en.wikipedia.org/wiki/Interval_(mathematics)
    template <typename T>
    class ValueRange
    {
    public:
        
        /// @brief Value type.
        /// @details Alias for the type of the value that this class was template
        ///   instantiated for.
        using value_type = T;
        
        /// @brief Limits alias for the <code>value_type</code>.
        using limits = std::numeric_limits<value_type>;
        
        /// @brief Gets the "lowest" value supported by the <code>value_type</code>.
        /// @return Negative infinity if supported by the value type, limits::lowest()
        ///   otherwise.
        static constexpr value_type GetLowest() noexcept
        {
            return (limits::has_infinity)? -limits::infinity(): limits::lowest();
        }
        
        /// @brief Gets the "highest" value supported by the <code>value_type</code>.
        /// @return Positive infinity if supported by the value type, limits::max()
        ///   otherwise.
        static constexpr value_type GetHighest() noexcept
        {
            return (limits::has_infinity)? limits::infinity(): limits::max();
        }

        /// @brief Default constructor.
        /// @details Constructs an "unset" value range.
        /// @post <code>GetMin()</code> returns the value of <code>GetHighest()</code>.
        /// @post <code>GetMax()</code> returns the value of <code>GetLowest()</code>.
        constexpr ValueRange() = default;
        
        /// @brief Copy constructor.
        /// @post <code>GetMin()</code> returns the value of <code>other.GetMin()</code>.
        /// @post <code>GetMax()</code> returns the value of <code>other.GetMax()</code>.
        constexpr ValueRange(const ValueRange& other) = default;

        /// @brief Move constructor.
        /// @post <code>GetMin()</code> returns the value of <code>other.GetMin()</code>.
        /// @post <code>GetMax()</code> returns the value of <code>other.GetMax()</code>.
        constexpr ValueRange(ValueRange&& other) = default;
        
        /// @brief Initializing constructor.
        /// @post <code>GetMin()</code> returns the value of <code>v</code>.
        /// @post <code>GetMax()</code> returns the value of <code>v</code>.
        constexpr explicit ValueRange(const value_type& v) noexcept:
            ValueRange(pair_type{v, v})
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr ValueRange(const value_type& a, const value_type& b) noexcept:
            ValueRange(std::minmax(a, b))
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr ValueRange(const std::initializer_list<T> ilist) noexcept:
            ValueRange(std::minmax(ilist))
        {
            // Intentionally empty.
        }
        
        ~ValueRange() noexcept = default;
        
        /// @brief Copy assignment operator.
        /// @post <code>GetMin()</code> returns the value of <code>other.GetMin()</code>.
        /// @post <code>GetMax()</code> returns the value of <code>other.GetMax()</code>.
        ValueRange& operator= (const ValueRange& other) = default;

        /// @brief Move assignment operator.
        /// @post <code>GetMin()</code> returns the value of <code>other.GetMin()</code>.
        /// @post <code>GetMax()</code> returns the value of <code>other.GetMax()</code>.
        ValueRange& operator= (ValueRange&& other) = default;
        
        /// @brief Moves the value range by the given amount.
        /// @warning Behavior is undefined if incrementing the min or max value by
        ///   the given amount overflows the finite range of the <code>value_type</code>,
        constexpr ValueRange& Move(const value_type& v) noexcept
        {
            m_min += v;
            m_max += v;
            return *this;
        }

        /// @brief Gets the minimum value of this range.
        constexpr value_type GetMin() const noexcept
        {
            return m_min;
        }

        /// @brief Gets the maximum value of this range.
        constexpr value_type GetMax() const noexcept
        {
            return m_max;
        }
        
        /// @brief Includes the given value into this value range.
        /// @note If this value is the "unset" value then the result of this operation
        ///   will be the given value.
        /// @param v Value to "include" into this value.
        /// @post This value's "min" is the minimum of the given value and this value's "min".
        constexpr ValueRange& Include(const value_type& v) noexcept
        {
            m_min = std::min(v, GetMin());
            m_max = std::max(v, GetMax());
            return *this;
        }
        
        /// @brief Includes the given value range into this value range.
        /// @note If this value is the "unset" value then the result of this operation
        ///   will be the given value.
        /// @param v Value to "include" into this value.
        /// @post This value's "min" is the minimum of the given value's "min" and
        ///   this value's "min".
        constexpr ValueRange& Include(const ValueRange& v) noexcept
        {
            m_min = std::min(v.GetMin(), GetMin());
            m_max = std::max(v.GetMax(), GetMax());
            return *this;
        }
        
        /// @brief Intersects this value range with the given value range.
        constexpr ValueRange& Intersect(const ValueRange& v) noexcept
        {
            const auto min = std::max(v.GetMin(), GetMin());
            const auto max = std::min(v.GetMax(), GetMax());
            *this = (min <= max)? pair_type{min, max}: ValueRange{};
            return *this;
        }
        
        /// @brief Expands this value range.
        /// @details Expands this value range by decreasing the min value if the
        ///   given value is negative, or by increasing the max value if the
        ///   given value is positive.
        /// @param v Amount to expand this value range by.
        /// @warning Behavior is undefined if expanding the range by
        ///   the given amount overflows the range of the <code>value_type</code>,
        constexpr ValueRange& Expand(const value_type& v) noexcept
        {
            if (v < value_type{})
            {
                m_min += v;
            }
            else
            {
                m_max += v;
            }
            return *this;
        }
        
        /// @brief Expands equally both ends of this value range.
        /// @details Expands equally this value range by decreasing the min value and
        ///   by increasing the max value by the given amount.
        /// @note This operation has no effect if this value range is "unset".
        /// @param v Amount to expand both ends of this value range by.
        /// @warning Behavior is undefined if expanding the range by
        ///   the given amount overflows the range of the <code>value_type</code>,
        constexpr ValueRange& ExpandEqually(const NonNegative<value_type>& v) noexcept
        {
            const auto amount = value_type{v};
            m_min -= amount;
            m_max += amount;
            return *this;
        }
        
    private:
        /// @brief Internal pair type.
        using pair_type = std::pair<value_type, value_type>;
        
        /// @brief Internal pair type accepting constructor.
        constexpr explicit ValueRange(pair_type pair) noexcept:
            m_min{pair.first}, m_max{pair.second}
        {
            // Intentionally empty.
        }

        value_type m_min = GetHighest();
        value_type m_max = GetLowest();
    };
    
    /// @brief Gets the size of the given value range.
    /// @details Gets the difference between the max and min values.
    /// @warning Behavior is undefined if the difference between the given range's
    ///   max and min values overflows the range of the <code>ValueRange::value_type</code>.
    /// @return Non-negative value unless the given value range is "unset" or invalid.
    template <typename T>
    constexpr T GetSize(const ValueRange<T>& v) noexcept
    {
        return v.GetMax() - v.GetMin();
    }
    
    /// @brief Gets the center of the given value range.
    /// @warning Behavior is undefined if the difference between the given range's
    ///   max and min values overflows the range of the <code>ValueRange::value_type</code>.
    /// @relatedalso ValueRange
    template <typename T>
    constexpr T GetCenter(const ValueRange<T>& v) noexcept
    {
        // Rounding may cause issues...
        return (v.GetMin() + v.GetMax()) / 2;
    }
    
    /// @brief Checks whether two value ranges have any intersection/overlap at all.
    /// @relatedalso ValueRange
    template <typename T>
    constexpr bool IsIntersecting(const ValueRange<T>& a, const ValueRange<T>& b) noexcept
    {
        const auto maxOfMins = std::max(a.GetMin(), b.GetMin());
        const auto minOfMaxs = std::min(a.GetMax(), b.GetMax());
        return minOfMaxs >= maxOfMins;
    }
    
    /// @brief Gets the intersecting value range of two given ranges.
    /// @relatedalso ValueRange
    template <typename T>
    constexpr ValueRange<T> GetIntersection(ValueRange<T> a, const ValueRange<T>& b) noexcept
    {
        return a.Intersect(b);
    }
    
    /// @brief Determines whether the first range is entirely before the second range.
    template <typename T>
    constexpr bool IsEntirelyBefore(const ValueRange<T>& a, const ValueRange<T>& b)
    {
        return a.GetMax() < b.GetMin();
    }
    
    /// @brief Determines whether the first range is entirely after the second range.
    template <typename T>
    constexpr bool IsEntirelyAfter(const ValueRange<T>& a, const ValueRange<T>& b)
    {
        return a.GetMin() > b.GetMax();
    }
    
    /// @brief Determines whether the first range entirely encloses the second.
    template <typename T>
    constexpr bool IsEntirelyEnclosing(const ValueRange<T>& a, const ValueRange<T>& b)
    {
        return a.GetMin() <= b.GetMin() && a.GetMax() >= b.GetMax();
    }

    /// @brief Equality operator.
    /// @note Satisfies the EqualityComparable concept for ValueRange objects.
    /// @relatedalso ValueRange
    /// @sa http://en.cppreference.com/w/cpp/concept/EqualityComparable
    template <typename T>
    constexpr bool operator== (const ValueRange<T>& a, const ValueRange<T>& b) noexcept
    {
        return (a.GetMin() == b.GetMin()) && (a.GetMax() == b.GetMax());
    }
    
    /// @brief Inequality operator.
    /// @note Satisfies the EqualityComparable concept for ValueRange objects.
    /// @relatedalso ValueRange
    /// @sa http://en.cppreference.com/w/cpp/concept/EqualityComparable
    template <typename T>
    constexpr bool operator!= (const ValueRange<T>& a, const ValueRange<T>& b) noexcept
    {
        return !(a == b);
    }
    
    /// @brief Less-than operator.
    /// @note Provides a "strict weak ordering" relation.
    /// @note This is a lexicographical comparison.
    /// @note Obeys the LessThanComparable concept:
    ///   for all a, !(a < a); if (a < b) then !(b < a); if (a < b) and (b < c)
    ///   then (a < c); with equiv = !(a < b) && !(b < a), if equiv(a, b) and equiv(b, c),
    ///   then equiv(a, c).
    /// @relatedalso ValueRange
    /// @sa https://en.wikipedia.org/wiki/Weak_ordering#Strict_weak_orderings
    /// @sa http://en.cppreference.com/w/cpp/concept/LessThanComparable
    template <typename T>
    constexpr bool operator< (const ValueRange<T>& lhs, const ValueRange<T>& rhs) noexcept
    {
        return (lhs.GetMin() < rhs.GetMin()) ||
            (lhs.GetMin() == rhs.GetMin() && lhs.GetMax() < rhs.GetMax());
    }
    
    /// @brief Less-than or equal-to operator.
    /// @note Provides a "strict weak ordering" relation.
    /// @note This is a lexicographical comparison.
    /// @relatedalso ValueRange
    /// @sa https://en.wikipedia.org/wiki/Weak_ordering#Strict_weak_orderings
    template <typename T>
    constexpr bool operator<= (const ValueRange<T>& lhs, const ValueRange<T>& rhs) noexcept
    {
        return (lhs.GetMin() < rhs.GetMin()) ||
            (lhs.GetMin() == rhs.GetMin() && lhs.GetMax() <= rhs.GetMax());
    }
    
    /// @brief Greater-than operator.
    /// @note Provides a "strict weak ordering" relation.
    /// @note This is a lexicographical comparison.
    /// @relatedalso ValueRange
    /// @sa https://en.wikipedia.org/wiki/Weak_ordering#Strict_weak_orderings
    template <typename T>
    constexpr bool operator> (const ValueRange<T>& lhs, const ValueRange<T>& rhs) noexcept
    {
        return (lhs.GetMin() > rhs.GetMin()) ||
            (lhs.GetMin() == rhs.GetMin() && lhs.GetMax() > rhs.GetMax());
    }
    
    /// @brief Greater-than or equal-to operator.
    /// @note Provides a "strict weak ordering" relation.
    /// @note This is a lexicographical comparison.
    /// @relatedalso ValueRange
    /// @sa https://en.wikipedia.org/wiki/Weak_ordering#Strict_weak_orderings
    template <typename T>
    constexpr bool operator>= (const ValueRange<T>& lhs, const ValueRange<T>& rhs) noexcept
    {
        return (lhs.GetMin() > rhs.GetMin()) ||
            (lhs.GetMin() == rhs.GetMin() && lhs.GetMax() >= rhs.GetMax());
    }
    
    /// @brief Output stream operator.
    template <typename T>
    ::std::ostream& operator<< (::std::ostream& os, const ValueRange<T>& value)
    {
        return os << '{' << value.GetMin() << "..." << value.GetMax() << '}';
    }

} // namespace playrho

#endif // PLAYRHO_COMMON_VALUERANGE_HPP
