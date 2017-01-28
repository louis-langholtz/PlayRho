/*
 * Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef FixedPoint32_hpp
#define FixedPoint32_hpp

#include <cstdint>
#include <limits>
#include <cassert>
#include <cmath>

#include <Box2D/Common/Wider.hpp>

namespace box2d
{
	/// Fixed.
	///
	/// @detail This is a fixed point type template for a given base type using a given number
	/// of fraction bits.
	///
	/// This is a 32-bit sized fixed point type with a 18.14 format.
	/// With a 14-bit fraction part:
	///   * 0.000061035156250 is the smallest double precision value that can be represented;
	///   *
	template <typename BASE_TYPE, unsigned int FRACTION_BITS>
	class Fixed
	{
	public:
		using value_type = BASE_TYPE;
		static constexpr unsigned int FractionBits = FRACTION_BITS;

		static constexpr Fixed GetMin() noexcept
		{
			return Fixed{internal_type{1}};
		}
		
		static constexpr Fixed GetInfinity() noexcept
		{
			return Fixed{internal_type{numeric_limits::max()}};
		}
		
		static constexpr Fixed GetNegativeInfinity() noexcept
		{
			return Fixed{internal_type{numeric_limits::lowest()}};
		}
		
		static constexpr Fixed GetMax() noexcept
		{
			return Fixed{internal_type{numeric_limits::max() - 1}};
		}
		
		static constexpr Fixed GetLowest() noexcept
		{
			return Fixed{internal_type{numeric_limits::lowest() + 1}};
		}

		Fixed() = default;

		constexpr Fixed(const long double val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed(const double val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed(const unsigned long long val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
		}

		constexpr Fixed(const unsigned long val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
		}
		
		constexpr Fixed(const unsigned int val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
		}

		constexpr Fixed(const long long val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed(const float val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}

		constexpr Fixed(const int val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			//assert(val <= static_cast<decltype(val)>(GetMax()));
			//assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed(short val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
		}
		
		constexpr Fixed(value_type val, unsigned int fraction) noexcept:
			m_data{internal_type{static_cast<value_type>(static_cast<uint32_t>(val * ScaleFactor) | fraction)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
			assert(fraction <= (1u << FractionBits) - 1u);
		}
		
		// Unary operations

		explicit constexpr operator long double() const noexcept
		{
			return m_data.value / double(ScaleFactor);
		}
		
		explicit constexpr operator double() const noexcept
		{
			return m_data.value / double(ScaleFactor);
		}
		
		explicit constexpr operator float() const noexcept
		{
			return m_data.value / float(ScaleFactor);
		}
	
		explicit constexpr operator long long() const noexcept
		{
			return m_data.value / ScaleFactor;
		}
		
		explicit constexpr operator unsigned long long() const noexcept
		{
			assert(m_data.value >= 0);
			return static_cast<unsigned long long>(m_data.value / ScaleFactor);
		}

		explicit constexpr operator unsigned long() const noexcept
		{
			assert(m_data.value >= 0);
			return static_cast<unsigned long>(m_data.value / ScaleFactor);
		}
		
		explicit constexpr operator unsigned int() const noexcept
		{
			assert(m_data.value >= 0);
			return static_cast<unsigned int>(m_data.value / ScaleFactor);
		}

		explicit constexpr operator int() const noexcept
		{
			return static_cast<int>(m_data.value / ScaleFactor);
		}
		
		explicit constexpr operator short() const noexcept
		{
			return static_cast<short>(m_data.value / ScaleFactor);
		}
		
		constexpr Fixed operator- () const noexcept
		{
			return Fixed{internal_type{-m_data.value}};
		}
		
		constexpr Fixed operator+ () const noexcept
		{
			return Fixed{internal_type{+m_data.value}};
		}
		
		explicit constexpr operator bool() const noexcept
		{
			return m_data.value != 0;
		}
		
		constexpr bool operator! () const noexcept
		{
			return m_data.value == 0;
		}
		
		constexpr Fixed& operator+= (Fixed val) noexcept
		{
			assert(is_valid());
			assert(val.is_valid());

			const auto result = intermediary_type{m_data.value} + val.m_data.value;
			
			assert(result <= GetMax().m_data.value);
			assert(result >= GetLowest().m_data.value);

			if (result > GetMax().m_data.value)
			{
				m_data.value = GetInfinity().m_data.value;
			}
			else if (result < GetLowest().m_data.value)
			{
				m_data.value = GetNegativeInfinity().m_data.value;
			}
			else
			{
				m_data.value = static_cast<value_type>(result);
			}
			return *this;
		}

		constexpr Fixed& operator-= (Fixed val) noexcept
		{
			assert(is_valid());
			assert(val.is_valid());

			const auto result = intermediary_type{m_data.value} - val.m_data.value;
			
			assert(result <= GetMax().m_data.value);
			assert(result >= GetLowest().m_data.value);

			if (result > GetMax().m_data.value)
			{
				m_data.value = GetInfinity().m_data.value;
			}
			else if (result < GetLowest().m_data.value)
			{
				m_data.value = GetNegativeInfinity().m_data.value;
			}
			else
			{
				m_data.value = static_cast<value_type>(result);
			}
			return *this;
		}

		constexpr Fixed& operator*= (Fixed val) noexcept
		{
			assert(is_valid());
			assert(val.is_valid());

			const auto product = intermediary_type{m_data.value} * intermediary_type{val.m_data.value};
			const auto result = (product + ScaleFactor/2) / ScaleFactor;
			
			assert(result <= GetMax().m_data.value);
			assert(result >= GetLowest().m_data.value);
			
			m_data.value = static_cast<value_type>(result);
			return *this;
		}

		constexpr Fixed& operator/= (Fixed val) noexcept
		{
			assert(is_valid());
			assert(val.is_valid());

			const auto product = intermediary_type{m_data.value} * ScaleFactor;
			const auto result = product / val.m_data.value;
			
			assert(result <= GetMax().m_data.value);
			assert(result >= GetLowest().m_data.value);
			
			m_data.value = static_cast<value_type>(result);
			return *this;
		}
		
		constexpr Fixed& operator%= (Fixed val) noexcept
		{
			assert(is_valid());
			assert(val.is_valid());

			m_data.value %= val.m_data.value;
			return *this;
		}

		// Comparison operators

		constexpr int Compare(const Fixed other) const noexcept
		{
			if (m_data.value < other.m_data.value)
			{
				return -1;
			}
			if (m_data.value > other.m_data.value)
			{
				return +1;
			}
			return 0;
		}
		
	private:
		static constexpr value_type ScaleFactor = static_cast<value_type>(1u << FractionBits);

		using intermediary_type = typename Wider<value_type>::type;
	
		struct internal_type
		{
			value_type value;
		};
		
		using numeric_limits = std::numeric_limits<value_type>;
		
		constexpr Fixed(internal_type val) noexcept:
			m_data{val}
		{
			// Intentionally empty.
		}
		
		constexpr bool is_valid() const noexcept
		{
			return (m_data.value > GetNegativeInfinity().m_data.value)
				&& (m_data.value < GetInfinity().m_data.value);
		}
		
		internal_type m_data;
	};
	
	using Fixed32 = Fixed<std::int32_t,14>;
	using Fixed64 = Fixed<std::int64_t,16>;

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
		return lhs.Compare(rhs) == 0;
	}
	
	
	constexpr bool operator!= (Fixed32 lhs, Fixed32 rhs) noexcept
	{
		return lhs.Compare(rhs) != 0;
	}
	
	
	constexpr bool operator <= (Fixed32 lhs, Fixed32 rhs) noexcept
	{
		return lhs.Compare(rhs) <= 0;
	}
	
	
	constexpr bool operator >= (Fixed32 lhs, Fixed32 rhs) noexcept
	{
		return lhs.Compare(rhs) >= 0;
	}
	
	
	constexpr bool operator < (Fixed32 lhs, Fixed32 rhs) noexcept
	{
		return lhs.Compare(rhs) < 0;
	}
	
	
	constexpr bool operator > (Fixed32 lhs, Fixed32 rhs) noexcept
	{
		return lhs.Compare(rhs) > 0;
	}

	// Fixed64 free functions.
	
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
		return lhs.Compare(rhs) == 0;
	}
	
	
	constexpr bool operator!= (Fixed64 lhs, Fixed64 rhs) noexcept
	{
		return lhs.Compare(rhs) != 0;
	}
	
	
	constexpr bool operator <= (Fixed64 lhs, Fixed64 rhs) noexcept
	{
		return lhs.Compare(rhs) <= 0;
	}
	
	
	constexpr bool operator >= (Fixed64 lhs, Fixed64 rhs) noexcept
	{
		return lhs.Compare(rhs) >= 0;
	}
	
	
	constexpr bool operator < (Fixed64 lhs, Fixed64 rhs) noexcept
	{
		return lhs.Compare(rhs) < 0;
	}
	
	
	constexpr bool operator > (Fixed64 lhs, Fixed64 rhs) noexcept
	{
		return lhs.Compare(rhs) > 0;
	}

} // namespace box2d

namespace std
{
	template <>
	class numeric_limits<box2d::Fixed32>
	{
	public:
		static constexpr bool is_specialized = true;
		
		static constexpr box2d::Fixed32 min() noexcept { return box2d::Fixed32::GetMin(); }
		static constexpr box2d::Fixed32 max() noexcept    { return box2d::Fixed32::GetMax(); }
		static constexpr box2d::Fixed32 lowest() noexcept { return box2d::Fixed32::GetLowest(); }
		
		static constexpr int digits = 31 - box2d::Fixed32::FractionBits;
		static constexpr int digits10 = 31 - box2d::Fixed32::FractionBits;
		static constexpr int max_digits10 = 5; // TODO: check this
		
		static constexpr bool is_signed = true;
		static constexpr bool is_integer = false;
		static constexpr bool is_exact = true;
		static constexpr int radix = 0;
		static constexpr box2d::Fixed32 epsilon() noexcept { return box2d::Fixed32{0}; } // TODO
		static constexpr box2d::Fixed32 round_error() noexcept { return box2d::Fixed32{0}; } // TODO
		
		static constexpr int min_exponent = 0;
		static constexpr int min_exponent10 = 0;
		static constexpr int max_exponent = 0;
		static constexpr int max_exponent10 = 0;
		
		static constexpr bool has_infinity = false;
		static constexpr bool has_quiet_NaN = false;
		static constexpr bool has_signaling_NaN = false;
		static constexpr float_denorm_style has_denorm = denorm_absent;
		static constexpr bool has_denorm_loss = false;
		static constexpr box2d::Fixed32 infinity() noexcept { return box2d::Fixed32{0}; }
		static constexpr box2d::Fixed32 quiet_NaN() noexcept { return box2d::Fixed32{0}; }
		static constexpr box2d::Fixed32 signaling_NaN() noexcept { return box2d::Fixed32{0}; }
		static constexpr box2d::Fixed32 denorm_min() noexcept { return box2d::Fixed32{0}; }
		
		static constexpr bool is_iec559 = false;
		static constexpr bool is_bounded = true;
		static constexpr bool is_modulo = false;
		
		static constexpr bool traps = false;
		static constexpr bool tinyness_before = false;
		static constexpr float_round_style round_style = round_toward_zero;
	};

	// Fixed32

	inline box2d::Fixed32 abs(box2d::Fixed32 value) noexcept
	{
		return (value < box2d::Fixed32{0})? -value: value;
	}
	
	inline box2d::Fixed32 sqrt(box2d::Fixed32 value)
	{
		return box2d::Fixed32{::std::sqrt(static_cast<float>(value))};
	}

	inline float atan2(box2d::Fixed32 y, box2d::Fixed32 x)
	{
		return atan2(static_cast<float>(y), static_cast<float>(x));
	}
	
	inline box2d::Fixed32 round(box2d::Fixed32 value) noexcept
	{
		return box2d::Fixed32{static_cast<int16_t>(value + (box2d::Fixed32{1} / box2d::Fixed32{2}))};
	}
	
	inline box2d::Fixed32 nextafter(box2d::Fixed32 from, box2d::Fixed32 to) noexcept
	{
		if (from < to)
		{
			return from + numeric_limits<box2d::Fixed32>::min();
		}
		if (from > to)
		{
			return from - numeric_limits<box2d::Fixed32>::min();
		}
		return to;
	}
	
	inline float cos(box2d::Fixed32 value)
	{
		return static_cast<float>(cos(static_cast<double>(value)));
	}
	
	inline float sin(box2d::Fixed32 value)
	{
		return static_cast<float>(sin(static_cast<double>(value)));
	}

	inline double exp(box2d::Fixed32 value)
	{
		return exp(static_cast<double>(value));
	}
	
	// Fixed64

	inline box2d::Fixed64 abs(box2d::Fixed64 value) noexcept
	{
		return (value < box2d::Fixed64{0})? -value: value;
	}
	
	inline box2d::Fixed64 sqrt(box2d::Fixed64 value)
	{
		return box2d::Fixed64{::std::sqrt(static_cast<double>(value))};
	}
	
	inline double atan2(box2d::Fixed64 y, box2d::Fixed64 x)
	{
		return atan2(static_cast<double>(y), static_cast<double>(x));
	}
	
	inline box2d::Fixed64 round(box2d::Fixed64 value) noexcept
	{
		return box2d::Fixed64{static_cast<int16_t>(value + (box2d::Fixed64{1} / box2d::Fixed64{2}))};
	}
	
	inline box2d::Fixed64 nextafter(box2d::Fixed64 from, box2d::Fixed64 to) noexcept
	{
		if (from < to)
		{
			return from + numeric_limits<box2d::Fixed64>::min();
		}
		if (from > to)
		{
			return from - numeric_limits<box2d::Fixed64>::min();
		}
		return to;
	}
	
	inline double cos(box2d::Fixed64 value)
	{
		return cos(static_cast<double>(value));
	}
	
	inline double sin(box2d::Fixed64 value)
	{
		return sin(static_cast<double>(value));
	}
	
	inline double exp(box2d::Fixed64 value)
	{
		return exp(static_cast<double>(value));
	}

} // namespace std

#endif /* FixedPoint32_hpp */
