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

#ifndef BoundedValue_hpp
#define BoundedValue_hpp

#include <PlayRho/Common/InvalidArgument.hpp>

#include <limits>
#include <type_traits>
#include <iostream>

namespace playrho {
    
    enum class LoValueCheck
    {
        Any,
        AboveZero,
        ZeroOrMore,
        AboveNegInf,
        NonZero
    };

    enum class HiValueCheck
    {
        Any,
        BelowZero,
    	ZeroOrLess,
        OneOrLess,
        BelowPosInf
    };

    template <typename T, class Enable = void>
    struct ValueCheckHelper
    {
        static constexpr bool has_one = false;
        static constexpr T one() noexcept { return T(0); }
    };
    
    template <typename T>
    struct ValueCheckHelper<T, typename std::enable_if<std::is_arithmetic<T>::value>::type>
    {
        static constexpr bool has_one = true;
        static constexpr T one() noexcept { return T(1); }
    };

    template <typename T>
    constexpr void CheckIfAboveNegInf(typename std::enable_if<!std::is_pointer<T>::value, T>::type value)
    {
        if (std::numeric_limits<T>::has_infinity)
        {
            if (!(value > -std::numeric_limits<T>::infinity()))
            {
                throw InvalidArgument{"value not > -inf"};;
            }
        }
    }
    
    template <typename T>
    constexpr void CheckIfAboveNegInf(typename std::enable_if<std::is_pointer<T>::value, T>::type )
    {
        // Intentionally empty.
    }

    template <typename T, LoValueCheck lo, HiValueCheck hi>
    class BoundedValue
    {
    public:
        using value_type = T;
        using remove_pointer_type = typename std::remove_pointer<T>::type;
        using exception_type = InvalidArgument;
        using this_type = BoundedValue<value_type, lo, hi>;

        static constexpr LoValueCheck GetLoCheck() { return lo; }
        static constexpr HiValueCheck GetHiCheck() { return hi; }

        static constexpr void DoLoCheck(value_type value)
        {
            switch (GetLoCheck())
            {
                case LoValueCheck::Any:
                    return;
                case LoValueCheck::AboveZero:
                    if (!(value > value_type(0)))
                    {
                        throw exception_type{"value not > 0"};
                    }
                    return;
                case LoValueCheck::ZeroOrMore:
                    if (!(value >= value_type(0)))
                    {
                        throw exception_type{"value not >= 0"};
                    }
                    return;
                case LoValueCheck::AboveNegInf:
                    CheckIfAboveNegInf<T>(value);
                    return;
                case LoValueCheck::NonZero:
                    if (value == static_cast<value_type>(0))
                    {
                        throw exception_type{"value may not be 0"};
                    }
                    return;
            }
        }
        
        static constexpr void DoHiCheck(value_type value)
        {
            switch (GetHiCheck())
            {
                case HiValueCheck::Any:
                    return;
                case HiValueCheck::BelowZero:
                    if (!(value < value_type(0)))
                    {
                        throw exception_type{"value not < 0"};
                    }
                    return;
                case HiValueCheck::ZeroOrLess:
                    if (!(value <= value_type(0)))
                    {
                        throw exception_type{"value not <= 0"};
                    }
                    return;
                case HiValueCheck::OneOrLess:
                    if (!ValueCheckHelper<value_type>::has_one)
                    {
                        throw exception_type{"value's type does not have a trivial 1"};
                    }
                    if (!(value <= ValueCheckHelper<value_type>::one()))
                    {
                        throw exception_type{"value not <= 1"};
                    }
                    return;
                case HiValueCheck::BelowPosInf:
                    if (std::numeric_limits<value_type>::has_infinity)
                    {
                        if (!(value < +std::numeric_limits<value_type>::infinity()))
                        {
                            throw exception_type{"value not < +inf"};;
                        }
                    }
                    return;
            }
        }

        constexpr BoundedValue(value_type value): m_value{value}
        {
            DoLoCheck(value);
            DoHiCheck(value);
        }
        
        constexpr BoundedValue& operator= (const this_type& other) noexcept
        {
            m_value = other.m_value;
            return *this;
        }

        constexpr BoundedValue& operator= (const T& value)
        {
            DoLoCheck(value);
            DoHiCheck(value);
            m_value = value;
            return *this;
        }

        constexpr operator value_type () const
        {
            return m_value;
        }

        template <typename U = T>
        typename std::enable_if<std::is_pointer<U>::value, U>::type operator-> () const
        {
            return m_value;
        }

        template <typename U = T>
        typename std::enable_if<std::is_pointer<U>::value, remove_pointer_type>::type& operator* () const
        {
            return *m_value;
        }

    private:
        value_type m_value;
    };
    
    // Logical operations for BoundedValue<T, lo, hi> OP BoundedValue<T, lo, hi>

    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator== (const BoundedValue<T, lo, hi> lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} == T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator!= (const BoundedValue<T, lo, hi> lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} != T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator<= (const BoundedValue<T, lo, hi> lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} <= T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator>= (const BoundedValue<T, lo, hi> lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} >= T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator< (const BoundedValue<T, lo, hi> lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} < T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator> (const BoundedValue<T, lo, hi> lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} > T{rhs};
    }

    // Logical operations for BoundedValue<T, lo, hi> OP T

    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator== (const BoundedValue<T, lo, hi> lhs, const T rhs)
    {
        return T{lhs} == T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator!= (const BoundedValue<T, lo, hi> lhs, const T rhs)
    {
        return T{lhs} != T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator<= (const BoundedValue<T, lo, hi> lhs, const T rhs)
    {
        return T{lhs} <= T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator>= (const BoundedValue<T, lo, hi> lhs, const T rhs)
    {
        return T{lhs} >= T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator< (const BoundedValue<T, lo, hi> lhs, const T rhs)
    {
        return T{lhs} < T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator> (const BoundedValue<T, lo, hi> lhs, const T rhs)
    {
        return T{lhs} > T{rhs};
    }

    // Logical operations for T OP BoundedValue<T, lo, hi>

    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator== (const T lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} == T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator!= (const T lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} != T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator<= (const T lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} <= T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator>= (const T lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} >= T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator< (const T lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} < T{rhs};
    }
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    bool operator> (const T lhs, const BoundedValue<T, lo, hi> rhs)
    {
        return T{lhs} > T{rhs};
    }

    // Unary operations for BoundedValue<T, lo, hi>
    
    // Common useful aliases...

    template <typename T>
    using NonNegative = BoundedValue<T, LoValueCheck::ZeroOrMore, HiValueCheck::Any>;

    template <typename T>
    using NonPositive = BoundedValue<T, LoValueCheck::Any, HiValueCheck::ZeroOrLess>;

    template <typename T>
    using Positive = BoundedValue<T, LoValueCheck::AboveZero, HiValueCheck::Any>;

    template <typename T>
    using Negative = BoundedValue<T, LoValueCheck::Any, HiValueCheck::BelowZero>;

    template <typename T>
    using Finite = BoundedValue<T, LoValueCheck::AboveNegInf, HiValueCheck::BelowPosInf>;
    
    template <typename T>
    using NonZero = typename std::enable_if<!std::is_pointer<T>::value,
        BoundedValue<T, LoValueCheck::NonZero, HiValueCheck::Any>>::type;

    template <typename T>
    using NonNull = typename std::enable_if<std::is_pointer<T>::value,
        BoundedValue<T, LoValueCheck::NonZero, HiValueCheck::Any>>::type;
    
    template <typename T>
    using UnitInterval = BoundedValue<T, LoValueCheck::ZeroOrMore, HiValueCheck::OneOrLess>;
    
    template <typename T, LoValueCheck lo, HiValueCheck hi>
    ::std::ostream& operator<<(::std::ostream& os, const BoundedValue<T, lo, hi>& value)
    {
        return os << T(value);
    }
}

#endif /* BoundedValue_hpp */
