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

namespace box2d
{
	class Fixed32
	{
	public:
		using value_type = int32_t;
		static constexpr unsigned FractionBits = 14;
		static constexpr uint32_t FractionMask = (1u << FractionBits) - 1;
		static constexpr value_type ScaleFactor = static_cast<value_type>(1 << FractionBits); // 2^16

		static constexpr Fixed32 GetMin() noexcept
		{
			return Fixed32{0, 1};
		}
		
		static constexpr Fixed32 GetMax() noexcept
		{
			return Fixed32{internal_type{numeric_limits::max()}};
		}
		
		static constexpr Fixed32 GetLowest() noexcept
		{
			return Fixed32{internal_type{numeric_limits::lowest()}};
		}

		Fixed32() = default;

		constexpr Fixed32(long double val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed32(double val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed32(unsigned long long val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
		}

		constexpr Fixed32(unsigned long val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
		}
		
		constexpr Fixed32(unsigned int val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
		}

		constexpr Fixed32(long long val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed32(float val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}

		constexpr Fixed32(int val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
		}
		
		constexpr Fixed32(short val) noexcept:
			m_data{internal_type{static_cast<value_type>(val * ScaleFactor)}}
		{
		}
		
		constexpr Fixed32(int32_t val, uint32_t fraction) noexcept:
			m_data{internal_type{static_cast<value_type>(static_cast<uint32_t>(val * ScaleFactor) | fraction)}}
		{
			assert(val <= static_cast<decltype(val)>(GetMax()));
			assert(val >= static_cast<decltype(val)>(GetLowest()));
			assert(fraction < ScaleFactor);
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
			return m_data.value / ScaleFactor;
		}
		
		explicit constexpr operator short() const noexcept
		{
			return static_cast<short>(m_data.value / ScaleFactor);
		}
		
		constexpr Fixed32 operator- () const noexcept
		{
			return Fixed32{internal_type{-m_data.value}};
		}
		
		constexpr Fixed32 operator+ () const noexcept
		{
			return Fixed32{internal_type{+m_data.value}};
		}
		
		constexpr Fixed32& operator+= (Fixed32 val) noexcept
		{
			// TODO bounds check for over or under flow
			m_data.value += val.m_data.value;
			return *this;
		}

		constexpr Fixed32& operator-= (Fixed32 val) noexcept
		{
			// TODO bounds check for over or under flow
			m_data.value -= val.m_data.value;
			return *this;
		}

		constexpr Fixed32& operator*= (Fixed32 val) noexcept
		{
			// TODO bounds check for over or under flow
			const auto product = intermediary_type{m_data.value} * intermediary_type{val.m_data.value};
			const auto res = (product + ScaleFactor/2) / ScaleFactor;
			
			assert(res <= numeric_limits::max());
			assert(res >= numeric_limits::lowest());
			
			m_data.value = static_cast<value_type>(res);
			return *this;
		}

		constexpr Fixed32& operator/= (Fixed32 val) noexcept
		{
			// TODO bounds check for over or under flow
			const auto product = intermediary_type{m_data.value} * ScaleFactor;
			const auto res = product / val.m_data.value;
			
			assert(res <= numeric_limits::max());
			assert(res >= numeric_limits::lowest());
			
			m_data.value = static_cast<value_type>(res);
			return *this;
		}
		
		constexpr Fixed32& operator%= (Fixed32 val) noexcept
		{
			m_data.value %= val.m_data.value;
			return *this;
		}

		// Comparison operators

		constexpr int Compare(const Fixed32 other) const noexcept
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
		
		//friend bool operator== (Fixed32 lhs, Fixed32 rhs) noexcept;
		//friend bool operator!= (Fixed32 lhs, Fixed32 rhs) noexcept;
		//friend bool operator<= (Fixed32 lhs, Fixed32 rhs) noexcept;
		//friend bool operator>= (Fixed32 lhs, Fixed32 rhs) noexcept;
		//friend bool operator< (Fixed32 lhs, Fixed32 rhs) noexcept;
		//friend bool operator> (Fixed32 lhs, Fixed32 rhs) noexcept;

	private:
		
		using intermediary_type = int64_t;
	
		struct internal_type
		{
			value_type value;
		};
		
		using numeric_limits = std::numeric_limits<value_type>;
		
		constexpr Fixed32(internal_type val) noexcept: m_data{val} {}
		
		internal_type m_data;
	};
	
	// Free functions.
	
	constexpr Fixed32 operator+ (Fixed32 lhs, Fixed32 rhs) noexcept
	{
		// TODO bounds check for over or under flow
		lhs += rhs;
		return lhs;
	}
	
	constexpr Fixed32 operator- (Fixed32 lhs, Fixed32 rhs) noexcept
	{
		// TODO bounds check for over or under flow
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
		// TODO bounds check for over or under flow
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

} // namespace box2d

namespace std
{
	template <>
	class numeric_limits<box2d::Fixed32>
	{
	public:
		static constexpr bool is_specialized = true;
		
		static constexpr box2d::Fixed32 min() noexcept { return box2d::Fixed32::GetMin(); }
		static constexpr box2d::Fixed32 max() noexcept { return box2d::Fixed32::GetMax(); }
		static constexpr box2d::Fixed32 lowest() noexcept { return box2d::Fixed32::GetLowest(); }
		
		static constexpr int digits = 31;
		static constexpr int digits10 = 0; // TODO
		static constexpr int max_digits10 = 5; // TODO: check this
		
		static constexpr bool is_signed = true;
		static constexpr bool is_integer = false;
		static constexpr bool is_exact = true;
		static constexpr int radix = 2; // TODO
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
		static constexpr bool is_bounded = true; // TODO: check this
		static constexpr bool is_modulo = false;
		
		static constexpr bool traps = true;
		static constexpr bool tinyness_before = false; // TODO
		static constexpr float_round_style round_style = round_toward_zero; // TODO
	};

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
} // namespace std

#endif /* FixedPoint32_hpp */
