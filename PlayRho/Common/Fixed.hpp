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

#ifndef PLAYRHO_COMMON_FIXED_HPP
#define PLAYRHO_COMMON_FIXED_HPP

#include <PlayRho/Common/Wider.hpp>
#include <PlayRho/Common/Templates.hpp>

#include <cstdint>
#include <limits>
#include <cassert>
#include <cmath>
#include <type_traits>
#include <iostream>

namespace playrho
{
    /// @brief A fixed-point template class.
    ///
    /// @details This is a fixed point type template for a given base type using a given number
    ///   of fraction bits.
    ///
    /// @note For a 32-bit sized fixed point type with a 14-bit fraction part
    ///   0.000061035156250 is the smallest double precision value that can be represented.
    ///
    template <typename BASE_TYPE, unsigned int FRACTION_BITS>
    class Fixed
    {
    public:
        
        /// @brief Value type.
        using value_type = BASE_TYPE;
        
        /// @brief Total number of bits.
        static constexpr unsigned int TotalBits = sizeof(BASE_TYPE) * 8;

        /// @brief Fraction bits.
        static constexpr unsigned int FractionBits = FRACTION_BITS;
        
        /// @brief Whole value bits.
        static constexpr unsigned int WholeBits = TotalBits - FractionBits;

        /// @brief Scale factor.
        static constexpr value_type ScaleFactor = static_cast<value_type>(1u << FractionBits);

        /// @brief Compare result enumeration.
        enum class CmpResult
        {
            Incomparable,
            Equal,
            LessThan,
            GreaterThan
        };

        /// @brief Gets the min value this type is capable of expressing.
        static constexpr Fixed GetMin() noexcept
        {
            return Fixed{1, scalar_type{1}};
        }
        
        /// @brief Gets an infinite value for this type.
        static constexpr Fixed GetInfinity() noexcept
        {
            return Fixed{numeric_limits::max(), scalar_type{1}};
        }
        
        /// @brief Gets the max value this type is capable of expressing.
        static constexpr Fixed GetMax() noexcept
        {
            // max reserved for +inf
            return Fixed{numeric_limits::max() - 1, scalar_type{1}};
        }

        /// @brief Gets a NaN value for this type.
        static constexpr Fixed GetNaN() noexcept
        {
            return Fixed{numeric_limits::lowest(), scalar_type{1}};
        }

        /// @brief Gets the negative infinity value for this type.
        static constexpr Fixed GetNegativeInfinity() noexcept
        {
            // lowest reserved for NaN
            return Fixed{numeric_limits::lowest() + 1, scalar_type{1}};
        }
        
        /// @brief Gets the lowest value this type is capable of expressing.
        static constexpr Fixed GetLowest() noexcept
        {
            // lowest reserved for NaN
            // lowest + 1 reserved for -inf
            return Fixed{numeric_limits::lowest() + 2, scalar_type{1}};
        }

        /// @brief Gets the value from a floating point value.
        template <typename T>
        static constexpr value_type GetFromFloat(T val) noexcept
        {
            static_assert(std::is_floating_point<T>::value, "floating point value required");
            // Note: std::isnan(val) *NOT* constant expression, so can't use here!
            return !(val <= 0 || val >= 0)? GetNaN().m_value:
                (val > static_cast<long double>(GetMax()))? GetInfinity().m_value:
                (val < static_cast<long double>(GetLowest()))? GetNegativeInfinity().m_value:
                static_cast<value_type>(val * ScaleFactor);
        }
        
        /// @brief Gets the value from a signed integral value.
        template <typename T>
        static constexpr value_type GetFromSignedInt(T val) noexcept
        {
            static_assert(std::is_integral<T>::value, "integral value required");
            static_assert(std::is_signed<T>::value, "must be signed");
            return (val > (GetMax().m_value / ScaleFactor))? GetInfinity().m_value:
                (val < (GetLowest().m_value / ScaleFactor))? GetNegativeInfinity().m_value:
                static_cast<value_type>(val * ScaleFactor);
        }
        
        /// @brief Gets the value from an unsigned integral value.
        template <typename T>
        static constexpr value_type GetFromUnsignedInt(T val) noexcept
        {
            static_assert(std::is_integral<T>::value, "integral value required");
            static_assert(!std::is_signed<T>::value, "must be unsigned");
            const auto max = static_cast<unsigned_wider_type>(GetMax().m_value / ScaleFactor);
            return (val > max)? GetInfinity().m_value: static_cast<value_type>(val) * ScaleFactor;
        }
        
        Fixed() = default;
        
        /// @brief Initializing constructor.
        constexpr Fixed(long double val) noexcept:
            m_value{GetFromFloat(val)}
        {
            // Intentionally empty
        }
        
        /// @brief Initializing constructor.
        constexpr Fixed(double val) noexcept:
            m_value{GetFromFloat(val)}
        {
            // Intentionally empty
        }

        /// @brief Initializing constructor.
        constexpr Fixed(float val) noexcept:
            m_value{GetFromFloat(val)}
        {
            // Intentionally empty
        }
        
        /// @brief Initializing constructor.
        constexpr Fixed(unsigned long long val) noexcept:
            m_value{GetFromUnsignedInt(val)}
        {
            // Intentionally empty.
        }

        /// @brief Initializing constructor.
        constexpr Fixed(unsigned long val) noexcept:
            m_value{GetFromUnsignedInt(val)}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr Fixed(unsigned int val) noexcept:
            m_value{GetFromUnsignedInt(val)}
        {
            // Intentionally empty.
        }

        /// @brief Initializing constructor.
        constexpr Fixed(long long val) noexcept:
            m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }

        /// @brief Initializing constructor.
        constexpr Fixed(long val) noexcept:
            m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr Fixed(int val) noexcept:
            m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr Fixed(short val) noexcept:
            m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        constexpr Fixed(value_type val, unsigned int fraction) noexcept:
            m_value{static_cast<value_type>(static_cast<std::uint32_t>(val * ScaleFactor) | fraction)}
        {
            // Intentionally empty.
        }
        
        /// @brief Initializing constructor.
        template <typename BT, unsigned int FB>
        constexpr Fixed(const Fixed<BT, FB> val) noexcept:
            Fixed(static_cast<long double>(val))
        {
            // Intentionally empty
        }
        
        // Methods
        
        /// @brief Converts the value to the expressed type.
        template <typename T>
        constexpr T ConvertTo() const noexcept
        {
            return isnan()? std::numeric_limits<T>::signaling_NaN():
                !isfinite()? std::numeric_limits<T>::infinity() * getsign():
                    m_value / static_cast<T>(ScaleFactor);
        }

        /// @brief Compares this value to the given one.
        constexpr CmpResult Compare(const Fixed other) const noexcept
        {
            if (isnan() || other.isnan())
            {
                return CmpResult::Incomparable;
            }
            if (m_value < other.m_value)
            {
                return CmpResult::LessThan;
            }
            if (m_value > other.m_value)
            {
                return CmpResult::GreaterThan;
            }
            return CmpResult::Equal;
        }

        // Unary operations

        /// @brief Long double operator.
        explicit constexpr operator long double() const noexcept
        {
            return ConvertTo<long double>();
        }
        
        /// @brief Double operator.
        explicit constexpr operator double() const noexcept
        {
            return ConvertTo<double>();
        }
        
        /// @brief Float operator.
        explicit constexpr operator float() const noexcept
        {
            return ConvertTo<float>();
        }
    
        /// @brief Long long operator.
        explicit constexpr operator long long() const noexcept
        {
            return m_value / ScaleFactor;
        }
        
        /// @brief Long operator.
        explicit constexpr operator long() const noexcept
        {
            return m_value / ScaleFactor;
        }

        /// @brief Unsigned long long operator.
        explicit constexpr operator unsigned long long() const noexcept
        {
            // Behavior is undefined if m_value is negative
            return static_cast<unsigned long long>(m_value / ScaleFactor);
        }

        /// @brief Unsigned long operator.
        explicit constexpr operator unsigned long() const noexcept
        {
            // Behavior is undefined if m_value is negative
            return static_cast<unsigned long>(m_value / ScaleFactor);
        }
        
        /// @brief Unsigned int operator.
        explicit constexpr operator unsigned int() const noexcept
        {
            // Behavior is undefined if m_value is negative
            return static_cast<unsigned int>(m_value / ScaleFactor);
        }

        /// @brief int operator.
        explicit constexpr operator int() const noexcept
        {
            return static_cast<int>(m_value / ScaleFactor);
        }
        
        /// @brief short operator.
        explicit constexpr operator short() const noexcept
        {
            return static_cast<short>(m_value / ScaleFactor);
        }
        
        /// @brief Negation operator.
        constexpr Fixed operator- () const noexcept
        {
            return (isnan())? *this: Fixed{-m_value, scalar_type{1}};
        }
        
        /// @brief Positive operator.
        constexpr Fixed operator+ () const noexcept
        {
            return *this;
        }
        
        /// @brief bool operator.
        explicit constexpr operator bool() const noexcept
        {
            return m_value != 0;
        }
        
        /// @brief Logical not operator.
        constexpr bool operator! () const noexcept
        {
            return m_value == 0;
        }
        
        /// @brief Addition assignment operator.
        constexpr Fixed& operator+= (Fixed val) noexcept
        {
            if (isnan() || val.isnan()
                || ((m_value == GetInfinity().m_value) && (val.m_value == GetNegativeInfinity().m_value))
                || ((m_value == GetNegativeInfinity().m_value) && (val.m_value == GetInfinity().m_value))
                )
            {
                *this = GetNaN();
            }
            else if (val.m_value == GetInfinity().m_value)
            {
                m_value = GetInfinity().m_value;
            }
            else if (val.m_value == GetNegativeInfinity().m_value)
            {
                m_value = GetNegativeInfinity().m_value;
            }
            else if (isfinite() && val.isfinite())
            {
                const auto result = wider_type{m_value} + val.m_value;
                if (result > GetMax().m_value)
                {
                    // overflow from max
                    m_value = GetInfinity().m_value;
                }
                else if (result < GetLowest().m_value)
                {
                    // overflow from lowest
                    m_value = GetNegativeInfinity().m_value;
                }
                else
                {
                    m_value = static_cast<value_type>(result);
                }
            }
            return *this;
        }

        /// @brief Subtraction assignment operator.
        constexpr Fixed& operator-= (Fixed val) noexcept
        {
            if (isnan() || val.isnan()
                || ((m_value == GetInfinity().m_value) && (val.m_value == GetInfinity().m_value))
                || ((m_value == GetNegativeInfinity().m_value) && (val.m_value == GetNegativeInfinity().m_value))
            )
            {
                *this = GetNaN();
            }
            else if (val.m_value == GetInfinity().m_value)
            {
                m_value = GetNegativeInfinity().m_value;
            }
            else if (val.m_value == GetNegativeInfinity().m_value)
            {
                m_value = GetInfinity().m_value;
            }
            else if (isfinite() && val.isfinite())
            {
                const auto result = wider_type{m_value} - val.m_value;
                if (result > GetMax().m_value)
                {
                    // overflow from max
                    m_value = GetInfinity().m_value;
                }
                else if (result < GetLowest().m_value)
                {
                    // overflow from lowest
                    m_value = GetNegativeInfinity().m_value;
                }
                else
                {
                    m_value = static_cast<value_type>(result);
                }
            }
            return *this;
        }

        /// @brief Multiplication assignment operator.
        constexpr Fixed& operator*= (Fixed val) noexcept
        {
            if (isnan() || val.isnan())
            {
                *this = GetNaN();
            }
            else if (!isfinite() || !val.isfinite())
            {
                if (m_value == 0 || val.m_value == 0)
                {
                    *this = GetNaN();
                }
                else
                {
                    *this = ((m_value > 0) != (val.m_value > 0))? -GetInfinity(): GetInfinity();
                }
            }
            else
            {
                const auto product = wider_type{m_value} * wider_type{val.m_value};
                const auto result = product / ScaleFactor;
                
                if (product != 0 && result == 0)
                {
                    // underflow
                    m_value = static_cast<value_type>(result);
                }
                else if (result > GetMax().m_value)
                {
                    // overflow from max
                    m_value = GetInfinity().m_value;
                }
                else if (result < GetLowest().m_value)
                {
                    // overflow from lowest
                    m_value = GetNegativeInfinity().m_value;
                }
                else
                {
                    m_value = static_cast<value_type>(result);
                }
            }
            return *this;
        }

        /// @brief Division assignment operator.
        constexpr Fixed& operator/= (Fixed val) noexcept
        {
            if (isnan() || val.isnan())
            {
                *this = GetNaN();
            }
            else if (!isfinite() && !val.isfinite())
            {
                *this = GetNaN();
            }
            else if (!isfinite())
            {
                *this = ((m_value > 0) != (val.m_value > 0))? -GetInfinity(): GetInfinity();
            }
            else if (!val.isfinite())
            {
                *this = 0;
            }
            else
            {
                const auto product = wider_type{m_value} * ScaleFactor;
                const auto result = product / val.m_value;
                
                if (product != 0 && result == 0)
                {
                    // underflow
                    m_value = static_cast<value_type>(result);
                }
                else if (result > GetMax().m_value)
                {
                    // overflow from max
                    m_value = GetInfinity().m_value;
                }
                else if (result < GetLowest().m_value)
                {
                    // overflow from lowest
                    m_value = GetNegativeInfinity().m_value;
                }
                else
                {
                    m_value = static_cast<value_type>(result);
                }
            }
            return *this;
        }
        
        /// @brief Modulo operator.
        constexpr Fixed& operator%= (Fixed val) noexcept
        {
            assert(!isnan());
            assert(!val.isnan());

            m_value %= val.m_value;
            return *this;
        }
        
        /// @brief Is finite.
        constexpr bool isfinite() const noexcept
        {
            return (m_value > GetNegativeInfinity().m_value)
            && (m_value < GetInfinity().m_value);
        }
        
        /// @brief Is NaN.
        constexpr bool isnan() const noexcept
        {
            return m_value == GetNaN().m_value;
        }
        
        /// @brief Gets this value's sign.
        constexpr int getsign() const noexcept
        {
            return (m_value >= 0)? +1: -1;
        }

    private:
        
        /// @brief Widened type alias.
        using wider_type = typename Wider<value_type>::type;

        /// @brief Unsigned widened type alias.
        using unsigned_wider_type = typename std::make_unsigned<wider_type>::type;

        /// @brief Scalar type.
        struct scalar_type
        {
            value_type value = 1; ///< Value.
        };
        
        /// @brief Numeric limits type alias.
        using numeric_limits = std::numeric_limits<value_type>;
        
        /// @brief Initializing constructor.
        constexpr Fixed(value_type val, scalar_type scalar) noexcept:
            m_value{val * scalar.value}
        {
            // Intentionally empty.
        }
        
        value_type m_value; ///< Value in internal form.
    };

    /// @brief Equality operator.
    template <typename BT, unsigned int FB>
    constexpr bool operator== (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed<BT, FB>::CmpResult::Equal;
    }
    
    /// @brief Inequality operator.
    template <typename BT, unsigned int FB>
    constexpr bool operator!= (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) != Fixed<BT, FB>::CmpResult::Equal;
    }
    
    /// @brief Less-than operator.
    template <typename BT, unsigned int FB>
    constexpr bool operator< (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed<BT, FB>::CmpResult::LessThan;
    }

    /// @brief Greater-than operator.
    template <typename BT, unsigned int FB>
    constexpr bool operator> (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed<BT, FB>::CmpResult::GreaterThan;
    }
    
    /// @brief Less-than or equal-to operator.
    template <typename BT, unsigned int FB>
    constexpr bool operator<= (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed<BT, FB>::CmpResult::LessThan ||
               result == Fixed<BT, FB>::CmpResult::Equal;
    }
    
    /// @brief Greater-than or equal-to operator.
    template <typename BT, unsigned int FB>
    constexpr bool operator>= (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed<BT, FB>::CmpResult::GreaterThan || result == Fixed<BT, FB>::CmpResult::Equal;
    }

    /// @brief Addition operator.
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator+ (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }
    
    /// @brief Subtraction operator.
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator- (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }
    
    /// @brief Multiplication operator.
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator* (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs *= rhs;
        return lhs;
    }
    
    /// @brief Division operator.
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator/ (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs /= rhs;
        return lhs;
    }
    
    /// @brief Modulo operator.
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator% (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs %= rhs;
        return lhs;
    }    

    /// @brief Square root's the given value.
    /// @note This is a specialization of the Sqrt template function for Fixed types.
    /// @note This implementation isn't meant to be fast, only correct enough.
    template <typename BT, unsigned int FB>
    inline auto Sqrt(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(std::sqrt(static_cast<long double>(arg)));
    }

    /// @brief Gets whether the given value is normal - i.e. not 0 nor infinite.
    template <typename BT, unsigned int FB>
    inline bool IsNormal(Fixed<BT, FB> arg)
    {
        return arg != Fixed<BT, FB>{0} && arg.isfinite();
    }
    
    /// @brief Computes the sine of the argument for Fixed types.
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> Sin(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(std::sin(static_cast<double>(arg)));
    }
    
    /// @brief Computes the cosine of the argument for Fixed types.
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> Cos(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(std::cos(static_cast<double>(arg)));
    }

    /// @brief Computes the square root of the sum of the squares.
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> Hypot(Fixed<BT, FB> x, Fixed<BT, FB> y)
    {
        return static_cast<Fixed<BT, FB>>(std::hypot(static_cast<double>(x), static_cast<double>(y)));
    }
    
    /// @brief Rounds the given value.
    /// @sa http://en.cppreference.com/w/cpp/numeric/math/round
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> Round(Fixed<BT, FB> value) noexcept
    {
        const auto tmp = value + (Fixed<BT, FB>{1} / Fixed<BT, FB>{2});
        const auto truncated = static_cast<typename Fixed<BT, FB>::value_type>(tmp);
        return Fixed<BT, FB>{truncated, 0};
    }
    
    /// @brief Truncates the given value.
    /// @sa http://en.cppreference.com/w/c/numeric/math/trunc
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> Trunc(Fixed<BT, FB> arg)
    {
        return static_cast<Fixed<BT, FB>>(static_cast<long long>(arg));
    }
    
    /// @brief Determines whether the given value is negative.
    template <typename BT, unsigned int FB>
    inline bool SignBit(Fixed<BT, FB> value) noexcept
    {
        return value.getsign() < 0;
    }
    
    /// @brief Gets whether the given value is not-a-number.
    template <typename BT, unsigned int FB>
    constexpr inline bool IsNan(Fixed<BT, FB> value) noexcept
    {
        return value.Compare(0) == Fixed<BT, FB>::CmpResult::Incomparable;
    }
    
    /// @brief Gets whether the given value is finite.
    template <typename BT, unsigned int FB>
    inline bool IsFinite(Fixed<BT, FB> value) noexcept
    {
        return (value > Fixed<BT, FB>::GetNegativeInfinity())
            && (value < Fixed<BT, FB>::GetInfinity());
    }
    
    /// @brief Computes the rounded value of the given value.
    template <typename BT, unsigned int FB>
    inline Fixed<BT, FB> RoundOff(Fixed<BT, FB> value, std::uint32_t precision = 100000)
    {
        const auto factor = Fixed<BT, FB>(precision);
        return Round(value * factor) / factor;
    }

    /// @brief Gets whether a given value is almost zero.
    /// @details An almost zero value is "subnormal". Dividing by these values can lead to
    /// odd results like a divide by zero trap occurring.
    /// @return <code>true</code> if the given value is almost zero, <code>false</code> otherwise.
    template <typename BT, unsigned int FB>
    constexpr inline bool AlmostZero(Fixed<BT, FB> value)
    {
        return value == 0;
    }

    /// @brief Determines whether the given two values are "almost equal".
    template <typename BT, unsigned int FB>
    constexpr inline bool AlmostEqual(Fixed<BT, FB> x, Fixed<BT, FB> y, int ulp = 2)
    {
        return Abs(x - y) <= Fixed<BT, FB>{0, static_cast<std::uint32_t>(ulp)};
    }

    /// @brief Output stream operator.
    template <typename BT, unsigned int FB>
    inline ::std::ostream& operator<<(::std::ostream& os, const Fixed<BT, FB>& value)
    {
        return os << static_cast<double>(value);
    }

    /// @brief 32-bit fixed precision type.
    ///
    /// @note The available numeric fidelity of any 32-bit fixed point type is very limited.
    ///   Using a 32-bit fixed point type for Real should only be considered for simulations
    ///   where it's been found to work and where the dynamics won't be changing between runs.
    ///
    using Fixed32 = Fixed<std::int32_t,9>;

    // Fixed32 free functions.
    
    /// @brief Addition operator.
    constexpr Fixed32 operator+ (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }

    /// @brief Subtraction operator.
    constexpr Fixed32 operator- (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }
    
    /// @brief Multiplication operator.
    constexpr Fixed32 operator* (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs *= rhs;
        return lhs;
    }
    
    /// @brief Division operator.
    constexpr Fixed32 operator/ (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs /= rhs;
        return lhs;
    }
    
    /// @brief Modulo operator.
    constexpr Fixed32 operator% (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs %= rhs;
        return lhs;
    }    
    
    /// @brief Equality operator.
    constexpr bool operator== (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed32::CmpResult::Equal;
    }
    
    /// @brief Inequality operator.
    constexpr bool operator!= (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        return lhs.Compare(rhs) != Fixed32::CmpResult::Equal;
    }
    
    /// @brief Less-than or equal-to operator.
    constexpr bool operator <= (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return (result == Fixed32::CmpResult::LessThan) || (result == Fixed32::CmpResult::Equal);
    }
    
    /// @brief Greater-than or equal-to operator.
    constexpr bool operator >= (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return (result == Fixed32::CmpResult::GreaterThan) || (result == Fixed32::CmpResult::Equal);
    }
    
    /// @brief Less-than operator.
    constexpr bool operator < (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed32::CmpResult::LessThan;
    }
    
    /// @brief Greater-than operator.
    constexpr bool operator > (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed32::CmpResult::GreaterThan;
    }

    /// @brief Gets an invalid value.
    template <>
    constexpr Fixed32 GetInvalid() noexcept
    {
        return Fixed32::GetNaN();
    }
    
    /// @brief Gets the specialized name for the Fixed32 type.
    /// @details Provides an interface to a specialized function for getting C-style
    ///   null-terminated array of characters that names the Fixed32 type.
    /// @return Non-null pointer to C-style string name of specified type.
    template <>
    inline const char* GetTypeName<Fixed32>() noexcept
    {
        return "Fixed32";
    }

#ifndef _WIN32
    // Fixed64 free functions.

    /// @brief 64-bit fixed precision type.
    using Fixed64 = Fixed<std::int64_t,24>;

    /// @brief Addition operator.
    constexpr Fixed64 operator+ (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }
    
    /// @brief Subtraction operator.
    constexpr Fixed64 operator- (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }
    
    /// @brief Multiplication operator.
    constexpr Fixed64 operator* (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs *= rhs;
        return lhs;
    }
    
    /// @brief Division operator.
    constexpr Fixed64 operator/ (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs /= rhs;
        return lhs;
    }
    
    constexpr Fixed64 operator% (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs %= rhs;
        return lhs;
    }
    
    /// @brief Equality operator.
    constexpr bool operator== (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed64::CmpResult::Equal;
    }
    
    /// @brief Inequality operator.
    constexpr bool operator!= (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        return lhs.Compare(rhs) != Fixed64::CmpResult::Equal;
    }
    
    constexpr bool operator <= (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return (result == Fixed64::CmpResult::LessThan) || (result == Fixed64::CmpResult::Equal);
    }
    
    constexpr bool operator >= (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return (result == Fixed64::CmpResult::GreaterThan) || (result == Fixed64::CmpResult::Equal);
    }
    
    constexpr bool operator < (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed64::CmpResult::LessThan;
    }
    
    constexpr bool operator > (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed64::CmpResult::GreaterThan;
    }

    /// @brief Specialization of the Wider trait for Fixed32 type.
    template<> struct Wider<Fixed32> {
        using type = Fixed64; ///< Wider type.
    };

    /// @brief Gets an invalid value.
    template <>
    constexpr Fixed64 GetInvalid() noexcept
    {
        return Fixed64::GetNaN();
    }
    
    /// @brief Gets the specialized name for the Fixed64 type.
    /// @details Provides an interface to a specialized function for getting C-style
    ///   null-terminated array of characters that names the Fixed64 type.
    /// @return Non-null pointer to C-style string name of specified type.
    template <>
    inline const char* GetTypeName<Fixed64>() noexcept
    {
        return "Fixed64";
    }

#endif /* !_WIN32 */

} // namespace playrho

namespace std
{
    // Generic Fixed
    
    /// @brief Computes the arc tangent.
    /// @note Since C++11, std::atan2 may be specialized as:
    ///   <code>Promoted atan2(Arithmetic1 y, Arithmetic2 x)</code>. Where <code>Promoted</code>
    ///   must always be double for inputs of type Fixed.
    template <typename BT, unsigned int FB>
    inline double atan2(playrho::Fixed<BT, FB> y, playrho::Fixed<BT, FB> x)
    {
        return atan2(static_cast<double>(y), static_cast<double>(x));
    }
    
    /// @brief Next after function specialization for Fixed types.
    /// @note Since C++11, std::nextafter may be specialized as:
    ///   <code>Promoted nextafter(Arithmetic from, Arithmetic to)</code>.
    ///   Where <code>Promoted</code> must always be double for inputs of type Fixed.
    template <typename BT, unsigned int FB>
    inline double nextafter(playrho::Fixed<BT, FB> from, playrho::Fixed<BT, FB> to) noexcept
    {
        if (from < to)
        {
            return static_cast<double>(from + numeric_limits<playrho::Fixed<BT, FB>>::min());
        }
        if (from > to)
        {
            return static_cast<double>(from - numeric_limits<playrho::Fixed<BT, FB>>::min());
        }
        return static_cast<double>(to);
    }
    
    /// @brief Computes the floating point remainder.
    /// @note Since C++11, std::nextafter may be specialized as:
    ///   <code>Promoted fmod(Arithmetic1 x, Arithmetic2 y)</code>.
    ///   Where <code>Promoted</code> must always be double for inputs of type Fixed.
    template <typename BT, unsigned int FB>
    inline double fmod(playrho::Fixed<BT, FB> x, playrho::Fixed<BT, FB> y)
    {
        return fmod(static_cast<double>(x), static_cast<double>(y));
    }
    
    /// @brief Computes the square root of the sum of the squares.
    /// @note Since C++11, std::hypot may be specialized as:
    ///   <code>Promoted hypot(Arithmetic1 x, Arithmetic2 y)</code>.
    ///   Where <code>Promoted</code> must always be double for inputs of type Fixed.
    template <typename BT, unsigned int FB>
    inline double hypot(playrho::Fixed<BT, FB> x, playrho::Fixed<BT, FB> y)
    {
        return hypot(static_cast<double>(x), static_cast<double>(y));
    }
    
    /// @brief Template specialization of numeric limits for Fixed types.
    /// @sa http://en.cppreference.com/w/cpp/types/numeric_limits
    template <typename BT, unsigned int FB>
    class numeric_limits<playrho::Fixed<BT,FB>>
    {
    public:
        static constexpr bool is_specialized = true; ///< Type is specialized.
        
        /// @brief Gets the min value available for the type.
        static constexpr playrho::Fixed<BT,FB> min() noexcept { return playrho::Fixed<BT,FB>::GetMin(); }

        /// @brief Gets the max value available for the type.
        static constexpr playrho::Fixed<BT,FB> max() noexcept    { return playrho::Fixed<BT,FB>::GetMax(); }

        /// @brief Gets the lowest value available for the type.
        static constexpr playrho::Fixed<BT,FB> lowest() noexcept { return playrho::Fixed<BT,FB>::GetLowest(); }
        
        /// @brief Number of radix digits that can be represented.
        static constexpr int digits = playrho::Fixed<BT,FB>::WholeBits - 1;

        /// @brief Number of decimal digits that can be represented.
        static constexpr int digits10 = playrho::Fixed<BT,FB>::WholeBits - 1;
        
        /// @brief Number of decimal digits necessary to differentiate all values.
        static constexpr int max_digits10 = 5; // TODO(lou): check this
        
        static constexpr bool is_signed = true; ///< Identifies signed types.
        static constexpr bool is_integer = false; ///< Identifies integer types.
        static constexpr bool is_exact = true; ///< Identifies exact type.
        static constexpr int radix = 0; ///< Radix used by the type.

        /// @brief Gets the epsilon value for the type.
        static constexpr playrho::Fixed32 epsilon() noexcept { return playrho::Fixed<BT,FB>{0}; } // TODO(lou)
        
        /// @brief Gets the round error value for the type.
        static constexpr playrho::Fixed32 round_error() noexcept { return playrho::Fixed<BT,FB>{0}; } // TODO(lou)
        
        /// @brief One more than smallest negative power of the radix that's a valid
        ///    normalized floating-point value.
        static constexpr int min_exponent = 0;

        /// @brief Smallest negative power of ten that's a valid normalized floating-point value.
        static constexpr int min_exponent10 = 0;
        
        /// @brief One more than largest integer power of radix that's a valid finite
        ///   floating-point value.
        static constexpr int max_exponent = 0;
        
        /// @brief Largest integer power of 10 that's a valid finite floating-point value.
        static constexpr int max_exponent10 = 0;
        
        static constexpr bool has_infinity = true; ///< Whether can represent infinity.
        static constexpr bool has_quiet_NaN = true; ///< Whether can represent quiet-NaN.
        static constexpr bool has_signaling_NaN = false; ///< Whether can represent signaling-NaN.
        static constexpr float_denorm_style has_denorm = denorm_absent; ///< Denorm style used.
        static constexpr bool has_denorm_loss = false; ///< Has denorm loss amount.

        /// @brief Gets the infinite value for the type.
        static constexpr playrho::Fixed<BT,FB> infinity() noexcept { return playrho::Fixed<BT,FB>::GetInfinity(); }
        
        /// @brief Gets the quiet NaN value for the type.
        static constexpr playrho::Fixed<BT,FB> quiet_NaN() noexcept { return playrho::Fixed<BT,FB>::GetNaN(); }

        /// @brief Gets the signaling NaN value for the type.
        static constexpr playrho::Fixed<BT,FB> signaling_NaN() noexcept { return playrho::Fixed<BT,FB>{0}; }
        
        /// @brief Gets the denorm value for the type.
        static constexpr playrho::Fixed<BT,FB> denorm_min() noexcept { return playrho::Fixed<BT,FB>{0}; }
        
        static constexpr bool is_iec559 = false; ///< @brief Not an IEEE 754 floating-point type.
        static constexpr bool is_bounded = true; ///< Type bounded: has limited precision.
        static constexpr bool is_modulo = false; ///< Doesn't modulo arithmetic overflows.
        
        static constexpr bool traps = false; ///< Doesn't do traps.
        static constexpr bool tinyness_before = false; ///< Doesn't detect tinyness before rounding.
        static constexpr float_round_style round_style = round_toward_zero; ///< Rounds down.
    };
    
} // namespace std

#endif // PLAYRHO_COMMON_FIXED_HPP
