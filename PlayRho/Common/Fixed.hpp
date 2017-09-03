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

#ifndef Fixed_hpp
#define Fixed_hpp

#include <PlayRho/Common/Wider.hpp>
#include <PlayRho/Common/Templates.hpp>

#include <cstdint>
#include <limits>
#include <cassert>
#include <cmath>
#include <type_traits>

namespace playrho
{
    /// @brief Fixed point template class.
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
        using value_type = BASE_TYPE;
        static constexpr unsigned int FractionBits = FRACTION_BITS;
        static constexpr value_type ScaleFactor = static_cast<value_type>(1u << FractionBits);

        enum class CmpResult
        {
            Incomparable,
            Equal,
            LessThan,
            GreaterThan
        };

        static constexpr Fixed GetMin() noexcept
        {
            return Fixed{1, scalar_type{1}};
        }
        
        static constexpr Fixed GetInfinity() noexcept
        {
            return Fixed{(numeric_limits::max)(), scalar_type{1}};
        }
        
        static constexpr Fixed GetMax() noexcept
        {
            // max reserved for +inf
            return Fixed{(numeric_limits::max)() - 1, scalar_type{1}};
        }

        static constexpr Fixed GetNaN() noexcept
        {
            return Fixed{numeric_limits::lowest(), scalar_type{1}};
        }

        static constexpr Fixed GetNegativeInfinity() noexcept
        {
            // lowest reserved for NaN
            return Fixed{numeric_limits::lowest() + 1, scalar_type{1}};
        }
        
        static constexpr Fixed GetLowest() noexcept
        {
            // lowest reserved for NaN
            // lowest + 1 reserved for -inf
            return Fixed{numeric_limits::lowest() + 2, scalar_type{1}};
        }

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
        
        template <typename T>
        static constexpr value_type GetFromSignedInt(T val) noexcept
        {
            static_assert(std::is_integral<T>::value, "integral value required");
            static_assert(std::is_signed<T>::value, "must be signed");
            return (val > (GetMax().m_value / ScaleFactor))? GetInfinity().m_value:
                (val < (GetLowest().m_value / ScaleFactor))? GetNegativeInfinity().m_value:
                static_cast<value_type>(val * ScaleFactor);
        }
        
        template <typename T>
        static constexpr value_type GetFromUnsignedInt(T val) noexcept
        {
            static_assert(std::is_integral<T>::value, "integral value required");
            static_assert(!std::is_signed<T>::value, "must be unsigned");
            const auto max = static_cast<unsigned_wider_type>(GetMax().m_value / ScaleFactor);
            return (val > max)? GetInfinity().m_value: static_cast<value_type>(val) * ScaleFactor;
        }
        
        Fixed() = default;
        
        constexpr Fixed(long double val) noexcept:
            m_value{GetFromFloat(val)}
        {
            // Intentionally empty
        }
        
        constexpr Fixed(double val) noexcept:
            m_value{GetFromFloat(val)}
        {
            // Intentionally empty
        }

        constexpr Fixed(float val) noexcept:
            m_value{GetFromFloat(val)}
        {
            // Intentionally empty
        }
        
        constexpr Fixed(unsigned long long val) noexcept:
            m_value{GetFromUnsignedInt(val)}
        {
            // Intentionally empty.
        }

        constexpr Fixed(unsigned long val) noexcept:
            m_value{GetFromUnsignedInt(val)}
        {
            // Intentionally empty.
        }
        
        constexpr Fixed(unsigned int val) noexcept:
            m_value{GetFromUnsignedInt(val)}
        {
            // Intentionally empty.
        }

        constexpr Fixed(long long val) noexcept:
            m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }

        constexpr Fixed(long val) noexcept:
        m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }
        
        constexpr Fixed(int val) noexcept:
            m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }
        
        constexpr Fixed(short val) noexcept:
            m_value{GetFromSignedInt(val)}
        {
            // Intentionally empty.
        }
        
        constexpr Fixed(value_type val, unsigned int fraction) noexcept:
            m_value{static_cast<value_type>(static_cast<std::uint32_t>(val * ScaleFactor) | fraction)}
        {
            // Intentionally empty.
        }
        
        template <typename BT, unsigned int FB>
        constexpr Fixed(const Fixed<BT, FB> val) noexcept:
            Fixed(static_cast<long double>(val))
        {
            // Intentionally empty
        }
        
        // Methods
        
        template <typename T>
        constexpr T ConvertTo() const noexcept
        {
            return isnan()? std::numeric_limits<T>::signaling_NaN():
                !isfinite()? std::numeric_limits<T>::infinity() * getsign():
                    m_value / static_cast<T>(ScaleFactor);
        }

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

        explicit constexpr operator long double() const noexcept
        {
            return ConvertTo<long double>();
        }
        
        explicit constexpr operator double() const noexcept
        {
            return ConvertTo<double>();
        }
        
        explicit constexpr operator float() const noexcept
        {
            return ConvertTo<float>();
        }
    
        explicit constexpr operator long long() const noexcept
        {
            return m_value / ScaleFactor;
        }
        
        explicit constexpr operator long() const noexcept
        {
            return m_value / ScaleFactor;
        }

        explicit constexpr operator unsigned long long() const noexcept
        {
            // Behavior is undefined if m_value is negative
            return static_cast<unsigned long long>(m_value / ScaleFactor);
        }

        explicit constexpr operator unsigned long() const noexcept
        {
            // Behavior is undefined if m_value is negative
            return static_cast<unsigned long>(m_value / ScaleFactor);
        }
        
        explicit constexpr operator unsigned int() const noexcept
        {
            // Behavior is undefined if m_value is negative
            return static_cast<unsigned int>(m_value / ScaleFactor);
        }

        explicit constexpr operator int() const noexcept
        {
            return static_cast<int>(m_value / ScaleFactor);
        }
        
        explicit constexpr operator short() const noexcept
        {
            return static_cast<short>(m_value / ScaleFactor);
        }
        
        constexpr Fixed operator- () const noexcept
        {
            return (isnan())? *this: Fixed{-m_value, scalar_type{1}};
        }
        
        constexpr Fixed operator+ () const noexcept
        {
            return *this;
        }
        
        explicit constexpr operator bool() const noexcept
        {
            return m_value != 0;
        }
        
        constexpr bool operator! () const noexcept
        {
            return m_value == 0;
        }
        
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
        
        constexpr Fixed& operator%= (Fixed val) noexcept
        {
            assert(!isnan());
            assert(!val.isnan());

            m_value %= val.m_value;
            return *this;
        }
        
        constexpr bool isfinite() const noexcept
        {
            return (m_value > GetNegativeInfinity().m_value)
            && (m_value < GetInfinity().m_value);
        }
        
        constexpr bool isnan() const noexcept
        {
            return m_value == GetNaN().m_value;
        }
        
        constexpr int getsign() const noexcept
        {
            return (m_value >= 0)? +1: -1;
        }

    private:
        
        using wider_type = typename Wider<value_type>::type;
        using unsigned_wider_type = typename std::make_unsigned<wider_type>::type;

        struct scalar_type
        {
            value_type value = 1;
        };
        
        using numeric_limits = std::numeric_limits<value_type>;
        
        constexpr Fixed(value_type val, scalar_type scalar) noexcept:
            m_value{val * scalar.value}
        {
            // Intentionally empty.
        }
        
        value_type m_value;
    };

    template <typename BT, unsigned int FB>
    constexpr bool operator== (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed<BT, FB>::CmpResult::Equal;
    }
    
    template <typename BT, unsigned int FB>
    constexpr bool operator!= (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) != Fixed<BT, FB>::CmpResult::Equal;
    }
    
    template <typename BT, unsigned int FB>
    constexpr bool operator< (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed<BT, FB>::CmpResult::LessThan;
    }

    template <typename BT, unsigned int FB>
    constexpr bool operator> (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed<BT, FB>::CmpResult::GreaterThan;
    }
    
    template <typename BT, unsigned int FB>
    constexpr bool operator<= (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed<BT, FB>::CmpResult::LessThan || result == Fixed<BT, FB>::CmpResult::Equal;
    }
    
    template <typename BT, unsigned int FB>
    constexpr bool operator>= (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed<BT, FB>::CmpResult::GreaterThan || result == Fixed<BT, FB>::CmpResult::Equal;
    }

    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator+ (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }
    
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator- (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }
    
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator* (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs *= rhs;
        return lhs;
    }
    
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator/ (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs /= rhs;
        return lhs;
    }
    
    template <typename BT, unsigned int FB>
    constexpr Fixed<BT, FB> operator% (Fixed<BT, FB> lhs, Fixed<BT, FB> rhs) noexcept
    {
        lhs %= rhs;
        return lhs;
    }    
    
    /// @brief 32-bit fixed precision type.
    ///
    /// @note The available numeric fidelity of any 32-bit fixed point type is very limited.
    ///   Using a 32-bit fixed point type for Real should only be considered for simulations
    ///   where it's been found to work and where the dynamics won't be changing between runs.
    ///
    using Fixed32 = Fixed<std::int32_t,9>;

    // Fixed32 free functions.
    
    constexpr Fixed32 operator+ (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }

    constexpr Fixed32 operator- (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }
    
    constexpr Fixed32 operator* (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs *= rhs;
        return lhs;
    }
    
    constexpr Fixed32 operator/ (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs /= rhs;
        return lhs;
    }
    
    constexpr Fixed32 operator% (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        lhs %= rhs;
        return lhs;
    }    
    
    constexpr bool operator== (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed32::CmpResult::Equal;
    }
    
    constexpr bool operator!= (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        return lhs.Compare(rhs) != Fixed32::CmpResult::Equal;
    }
    
    constexpr bool operator <= (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return (result == Fixed32::CmpResult::LessThan) || (result == Fixed32::CmpResult::Equal);
    }
    
    constexpr bool operator >= (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return (result == Fixed32::CmpResult::GreaterThan) || (result == Fixed32::CmpResult::Equal);
    }
    
    constexpr bool operator < (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed32::CmpResult::LessThan;
    }
    
    constexpr bool operator > (Fixed32 lhs, Fixed32 rhs) noexcept
    {
        const auto result = lhs.Compare(rhs);
        return result == Fixed32::CmpResult::GreaterThan;
    }

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

    using Fixed64 = Fixed<std::int64_t,24>;

    constexpr Fixed64 operator+ (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }
    
    constexpr Fixed64 operator- (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }
    
    constexpr Fixed64 operator* (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        lhs *= rhs;
        return lhs;
    }
    
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
    
    constexpr bool operator== (Fixed64 lhs, Fixed64 rhs) noexcept
    {
        return lhs.Compare(rhs) == Fixed64::CmpResult::Equal;
    }
    
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

    template<> struct Wider<Fixed32> { using type = Fixed64; };

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
    
    template <typename BT, unsigned int FB>
    constexpr bool isnormal(playrho::Fixed<BT, FB> arg)
    {
        return arg != playrho::Fixed<BT, FB>{0} && arg.isfinite();
    }
    
    // Fixed32

    template <>
    class numeric_limits<playrho::Fixed32>
    {
    public:
        static constexpr bool is_specialized = true;
#pragma push_macro("min") 
#pragma push_macro("max") 
#undef min
#undef max
        static constexpr playrho::Fixed32 min() noexcept { return playrho::Fixed32::GetMin(); }
        static constexpr playrho::Fixed32 max() noexcept    { return playrho::Fixed32::GetMax(); }
#pragma pop_macro("min") 
#pragma pop_macro("max") 
        static constexpr playrho::Fixed32 lowest() noexcept { return playrho::Fixed32::GetLowest(); }
        
        static constexpr int digits = 31 - playrho::Fixed32::FractionBits;
        static constexpr int digits10 = 31 - playrho::Fixed32::FractionBits;
        static constexpr int max_digits10 = 5; // TODO: check this
        
        static constexpr bool is_signed = true;
        static constexpr bool is_integer = false;
        static constexpr bool is_exact = true;
        static constexpr int radix = 0;
        static constexpr playrho::Fixed32 epsilon() noexcept { return playrho::Fixed32{0}; } // TODO
        static constexpr playrho::Fixed32 round_error() noexcept { return playrho::Fixed32{0}; } // TODO
        
        static constexpr int min_exponent = 0;
        static constexpr int min_exponent10 = 0;
        static constexpr int max_exponent = 0;
        static constexpr int max_exponent10 = 0;
        
        static constexpr bool has_infinity = true;
        static constexpr bool has_quiet_NaN = true;
        static constexpr bool has_signaling_NaN = false;
        static constexpr float_denorm_style has_denorm = denorm_absent;
        static constexpr bool has_denorm_loss = false;
        static constexpr playrho::Fixed32 infinity() noexcept { return playrho::Fixed32::GetInfinity(); }
        static constexpr playrho::Fixed32 quiet_NaN() noexcept { return playrho::Fixed32::GetNaN(); }
        static constexpr playrho::Fixed32 signaling_NaN() noexcept { return playrho::Fixed32{0}; }
        static constexpr playrho::Fixed32 denorm_min() noexcept { return playrho::Fixed32{0}; }
        
        static constexpr bool is_iec559 = false;
        static constexpr bool is_bounded = true;
        static constexpr bool is_modulo = false;
        
        static constexpr bool traps = false;
        static constexpr bool tinyness_before = false;
        static constexpr float_round_style round_style = round_toward_zero;
    };

    inline playrho::Fixed32 abs(playrho::Fixed32 value) noexcept
    {
        return (value < playrho::Fixed32{0})? -value: value;
    }
    
    inline playrho::Fixed32 sqrt(playrho::Fixed32 value)
    {
        return playrho::Fixed32{::std::sqrt(static_cast<double>(value))};
    }

    inline float atan2(playrho::Fixed32 y, playrho::Fixed32 x)
    {
        return atan2(static_cast<float>(y), static_cast<float>(x));
    }
    
    inline playrho::Fixed32 round(playrho::Fixed32 value) noexcept
    {
        return playrho::Fixed32{static_cast<int16_t>(value + (playrho::Fixed32{1} / playrho::Fixed32{2}))};
    }
    
    inline playrho::Fixed32 nextafter(playrho::Fixed32 from, playrho::Fixed32 to) noexcept
    {
        if (from < to)
        {
            return from + (numeric_limits<playrho::Fixed32>::min)();
        }
        if (from > to)
        {
            return from - (numeric_limits<playrho::Fixed32>::min)();
        }
        return to;
    }
    
    inline float cos(playrho::Fixed32 value)
    {
        return static_cast<float>(cos(static_cast<double>(value)));
    }
    
    inline float sin(playrho::Fixed32 value)
    {
        return static_cast<float>(sin(static_cast<double>(value)));
    }

    inline double exp(playrho::Fixed32 value)
    {
        return exp(static_cast<double>(value));
    }
    
    inline bool isfinite(playrho::Fixed32 value) noexcept
    {
        return (value > playrho::Fixed32::GetNegativeInfinity()) && (value < playrho::Fixed32::GetInfinity());
    }
    
    constexpr inline bool isnan(playrho::Fixed32 value) noexcept
    {
        return value.Compare(0) == playrho::Fixed32::CmpResult::Incomparable;
    }
    
    inline playrho::Fixed32 fmod(playrho::Fixed32 x, playrho::Fixed32 y)
    {
        return playrho::Fixed32(fmod(static_cast<double>(x), static_cast<double>(y)));
    }

#ifndef _WIN32

    template <>
    class numeric_limits<playrho::Fixed64>
    {
    public:
        static constexpr bool is_specialized = true;
        
        static constexpr playrho::Fixed64 minimum() noexcept { return playrho::Fixed64::GetMin(); }
        static constexpr playrho::Fixed64 maximum() noexcept    { return playrho::Fixed64::GetMax(); }
        static constexpr playrho::Fixed64 lowest() noexcept { return playrho::Fixed64::GetLowest(); }
        
        static constexpr int digits = 63 - playrho::Fixed64::FractionBits;
        static constexpr int digits10 = 63 - playrho::Fixed64::FractionBits;
        static constexpr int max_digits10 = 10; // TODO: check this
        
        static constexpr bool is_signed = true;
        static constexpr bool is_integer = false;
        static constexpr bool is_exact = true;
        static constexpr int radix = 0;
        static constexpr playrho::Fixed64 epsilon() noexcept { return playrho::Fixed64{0}; } // TODO
        static constexpr playrho::Fixed64 round_error() noexcept { return playrho::Fixed64{0}; } // TODO
        
        static constexpr int min_exponent = 0;
        static constexpr int min_exponent10 = 0;
        static constexpr int max_exponent = 0;
        static constexpr int max_exponent10 = 0;
        
        static constexpr bool has_infinity = true;
        static constexpr bool has_quiet_NaN = true;
        static constexpr bool has_signaling_NaN = false;
        static constexpr float_denorm_style has_denorm = denorm_absent;
        static constexpr bool has_denorm_loss = false;
        static constexpr playrho::Fixed64 infinity() noexcept { return playrho::Fixed64::GetInfinity(); }
        static constexpr playrho::Fixed64 quiet_NaN() noexcept { return playrho::Fixed64::GetNaN(); }
        static constexpr playrho::Fixed64 signaling_NaN() noexcept { return playrho::Fixed64{0}; }
        static constexpr playrho::Fixed64 denorm_min() noexcept { return playrho::Fixed64{0}; }
        
        static constexpr bool is_iec559 = false;
        static constexpr bool is_bounded = true;
        static constexpr bool is_modulo = false;
        
        static constexpr bool traps = false;
        static constexpr bool tinyness_before = false;
        static constexpr float_round_style round_style = round_toward_zero;
    };

    inline playrho::Fixed64 abs(playrho::Fixed64 value) noexcept
    {
        return (value < playrho::Fixed64{0})? -value: value;
    }
    
    inline playrho::Fixed64 sqrt(playrho::Fixed64 value)
    {
        return playrho::Fixed64{::std::sqrt(static_cast<double>(value))};
    }
    
    inline double atan2(playrho::Fixed64 y, playrho::Fixed64 x)
    {
        return atan2(static_cast<double>(y), static_cast<double>(x));
    }
    
    inline playrho::Fixed64 round(playrho::Fixed64 value) noexcept
    {
        const auto tmp = value + (playrho::Fixed64{1} / playrho::Fixed64{2});
        const auto truncated = static_cast<playrho::Fixed64::value_type>(tmp);
        return playrho::Fixed64{truncated, 0};
    }
    
    inline playrho::Fixed64 nextafter(playrho::Fixed64 from, playrho::Fixed64 to) noexcept
    {
        if (from < to)
        {
            return from + numeric_limits<playrho::Fixed64>::minimum();
        }
        if (from > to)
        {
            return from - numeric_limits<playrho::Fixed64>::minimum();
        }
        return to;
    }
    
    inline double cos(playrho::Fixed64 value)
    {
        return cos(static_cast<double>(value));
    }
    
    inline double sin(playrho::Fixed64 value)
    {
        return sin(static_cast<double>(value));
    }
    
    inline double exp(playrho::Fixed64 value)
    {
        return exp(static_cast<double>(value));
    }

    inline bool isfinite(playrho::Fixed64 value) noexcept
    {
        return (value > playrho::Fixed64::GetNegativeInfinity()) && (value < playrho::Fixed64::GetInfinity());
    }

    constexpr inline bool isnan(playrho::Fixed64 value) noexcept
    {
        return value.Compare(0) == playrho::Fixed64::CmpResult::Incomparable;
    }
    
    inline playrho::Fixed64 fmod(playrho::Fixed64 x, playrho::Fixed64 y)
    {
        return playrho::Fixed64(fmod(static_cast<double>(x), static_cast<double>(y)));
    }

#endif /* _WIN32 */

} // namespace std

#endif /* Fixed_hpp */
