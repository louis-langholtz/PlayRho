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

namespace playrho {
    
    /// @brief Value range template type.
    /// @details This type encapsulates a min-max value range relationship.
    /// @invariant The min and max values can only be the result of
    ///   <code>std::minmax(a, b)</code> or the special values of positive and
    ///   negative infinity respectively indicating the "unset" value.
    template <typename T>
    class ValueRange
    {
    public:
        
        /// @brief Value type.
        /// @details Alias for the type of the value that this class was template
        ///   instantiated for.
        using value_type = T;
        
        /// @brief Default constructor.
        /// @details Constructs an "unset" value range.
        /// @post <code>GetMin()</code> returns positive infinity.
        /// @post <code>GetMax()</code> returns negative infinity.
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
            m_value{v, v}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr ValueRange(const value_type& a, const value_type& b) noexcept:
            m_value(std::minmax(a, b))
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr ValueRange(const std::initializer_list<T> ilist) noexcept:
            m_value(std::minmax(ilist))
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
        constexpr ValueRange& Move(const value_type& v) noexcept
        {
            m_value.first += v;
            m_value.second += v;
            return *this;
        }

        /// @brief Gets the minimum value of this range.
        constexpr value_type GetMin() const noexcept
        {
            return m_value.first;
        }

        /// @brief Gets the maximum value of this range.
        constexpr value_type GetMax() const noexcept
        {
            return m_value.second;
        }
        
        /// @brief Includes the given value into this value range.
        constexpr ValueRange& Include(const value_type& v) noexcept
        {
            m_value.first = std::min(v, GetMin());
            m_value.second = std::max(v, GetMax());
            return *this;
        }
        
        /// @brief Includes the given value range into this value range.
        constexpr ValueRange& Include(const ValueRange& v) noexcept
        {
            m_value.first = std::min(v.GetMin(), GetMin());
            m_value.second = std::max(v.GetMax(), GetMax());
            return *this;
        }
        
        /// @brief Intersects this value range with the given value range.
        constexpr ValueRange& Intersect(const ValueRange& v) noexcept
        {
            const auto min = std::max(v.GetMin(), GetMin());
            const auto max = std::min(v.GetMax(), GetMax());
            m_value = (min <= max)? pair_type{min, max}: GetDefaultValue();
            return *this;
        }
        
        /// @brief Expands this value range.
        /// @details Expands this value range by decreasing the min value if the
        ///   given value is negative, or by increasing the max value if the
        ///   given value is positive.
        /// @param v Amount to expand this value range by.
        constexpr ValueRange& Expand(const value_type& v) noexcept
        {
            if (v < value_type{})
            {
                m_value.first += v;
            }
            else
            {
                m_value.second += v;
            }
            return *this;
        }
        
        /// @brief Expands equally both ends of this value range.
        /// @details Expands equally this value range by decreasing the min value and
        ///   by increasing the max value by the given amount.
        /// @note This operation has no effect if this value range is "unset".
        /// @param v Amount to expand both ends of this value range by.
        constexpr ValueRange& ExpandEqually(const NonNegative<value_type>& v) noexcept
        {
            const auto amount = value_type{v};
            m_value.first -= amount;
            m_value.second += amount;
            return *this;
        }
        
    private:
        using pair_type = std::pair<value_type, value_type>;

        static constexpr pair_type GetDefaultValue()
        {
            return {
                +std::numeric_limits<value_type>::infinity(),
                -std::numeric_limits<value_type>::infinity()
            };
        }

        pair_type m_value = GetDefaultValue();
    };

    /// @brief Equality operator.
    template <typename T>
    constexpr bool operator== (const ValueRange<T>& a, const ValueRange<T>& b) noexcept
    {
        return (a.GetMin() == b.GetMin()) && (a.GetMax() == b.GetMax());
    }
    
    /// @brief Inequality operator.
    template <typename T>
    constexpr bool operator!= (const ValueRange<T>& a, const ValueRange<T>& b) noexcept
    {
        return !(a == b);
    }
    
    /// @brief Gets the size of the given value range.
    /// @details Gets the difference between the max and min values.
    /// @return Non-negative value unless the given value range is "unset" or invalid.
    template <typename T>
    constexpr T GetSize(const ValueRange<T>& v) noexcept
    {
        return v.GetMax() - v.GetMin();
    }
    
    /// @brief Gets the center of the given value range.
    /// @relatedalso ValueRange
    template <typename T>
    constexpr T GetCenter(const ValueRange<T>& v) noexcept
    {
        return (v.GetMin() + v.GetMax()) / 2;
    }
    
    /// @brief Checks whether two value ranges have any intersection/overlap at all.
    template <typename T>
    constexpr bool IsIntersecting(const ValueRange<T>& a, const ValueRange<T>& b) noexcept
    {
        const auto minVal = std::max(a.GetMin(), b.GetMin());
        const auto maxVal = std::min(a.GetMax(), b.GetMax());
        return minVal <= maxVal;
    }
    
    /// @brief Gets the intersecting value range of two given ranges.
    template <typename T>
    constexpr ValueRange<T> GetIntersection(ValueRange<T> a, const ValueRange<T>& b) noexcept
    {
        return a.Intersect(b);
    }
    
    /// @brief Determines whether the second value range is within the first.
    template <typename T>
    constexpr bool IsWithin(const ValueRange<T>& a, const ValueRange<T>& b)
    {
        return b.GetMin() >= a.GetMin() && b.GetMax() <= a.GetMax();
    }

} // namespace playrho

#endif // PLAYRHO_COMMON_VALUERANGE_HPP
