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

#ifndef UnitVec2_hpp
#define UnitVec2_hpp

#include <Box2D/Common/Settings.hpp>
#include <cmath>
#include <iostream>

namespace box2d
{
	struct Vec2;
	class Angle;

	class UnitVec2
	{
	public:
		using data_type = float_t;
		
		static constexpr UnitVec2 GetRight() noexcept
		{
			return UnitVec2{1, 0};
		}
		
		static constexpr UnitVec2 GetLeft() noexcept
		{
			return UnitVec2{-1, 0};
		}
		
		static constexpr UnitVec2 GetTop() noexcept
		{
			return UnitVec2{0, 1};
		}
		
		static constexpr UnitVec2 GetBottom() noexcept
		{
			return UnitVec2{0, -1};
		}
		
		static constexpr UnitVec2 GetDefaultFallback() noexcept
		{
			return UnitVec2{};
		}
		
		static constexpr UnitVec2 GetZero() noexcept
		{
			return UnitVec2{0, 0};
		}
		
		constexpr UnitVec2() noexcept
		{
			// Intentionally empty.
		}
		
		explicit UnitVec2(const Vec2& value, UnitVec2 fallback = GetDefaultFallback()) noexcept;
		
		explicit UnitVec2(const Angle& angle) noexcept;
		
		constexpr auto GetX() const noexcept
		{
			return m_x;
		}
		
		constexpr auto GetY() const noexcept
		{
			return m_y;
		}
		
		constexpr auto cos() const noexcept
		{
			return m_x;
		}
		
		constexpr auto sin() const noexcept
		{
			return m_y;
		}
#if 0
		constexpr operator Vec2() const
		{
			return Vec2{GetX(), GetY()};
		}
#endif
		constexpr inline UnitVec2 FlipXY() const noexcept
		{
			return UnitVec2{-GetX(), -GetY()};
		}
		
		constexpr inline UnitVec2 FlipX() const noexcept
		{
			return UnitVec2{-GetX(), GetY()};
		}
		
		constexpr inline UnitVec2 FlipY() const noexcept
		{
			return UnitVec2{GetX(), -GetY()};
		}
		
		constexpr inline UnitVec2 Rotate(UnitVec2 amount) const noexcept
		{
			return UnitVec2{
				GetX() * amount.GetX() - GetY() * amount.GetY(),
				GetY() * amount.GetX() + GetX() * amount.GetY()
			};
		}
		
		/// Gets a vector counter-clockwise (reverse-clockwise) perpendicular to this vector.
		/// @detail This returns the unit vector (-y, x).
		/// @return A counter-clockwise 90-degree rotation of this vector.
		/// @sa GetFwdPerpendicular.
		constexpr inline UnitVec2 GetRevPerpendicular() const noexcept
		{
			// See http://mathworld.wolfram.com/PerpendicularVector.html
			return UnitVec2{-m_y, m_x};
		}
		
		/// Gets a vector clockwise (forward-clockwise) perpendicular to this vector.
		/// @detail This returns the unit vector (y, -x).
		/// @return A clockwise 90-degree rotation of this vector.
		/// @sa GetRevPerpendicular.
		constexpr inline UnitVec2 GetFwdPerpendicular() const noexcept
		{
			// See http://mathworld.wolfram.com/PerpendicularVector.html
			return UnitVec2{m_y, -m_x};
		}
		
		constexpr inline UnitVec2 operator- () const noexcept
		{
			return UnitVec2{-m_x, -m_y};
		}
		
		constexpr inline UnitVec2 Absolute() const noexcept
		{
			return UnitVec2{std::abs(m_x), std::abs(m_y)};
		}
		
	private:
		constexpr UnitVec2(data_type x, data_type y) noexcept:
			m_x{x}, m_y{y}
		{
			// Intentionally empty.
		}
		
		data_type m_x = std::numeric_limits<data_type>::signaling_NaN();
		data_type m_y = std::numeric_limits<data_type>::signaling_NaN();
	};
		
	/// Gets the unit vector for the given value.
	/// @param value Value to get the unit vector for.
	/// @return value divided by its length if length not almost zero otherwise invalid value.
	/// @sa almost_equal.
	UnitVec2 GetUnitVector(const Vec2& value, UnitVec2 fallback = UnitVec2::GetDefaultFallback());	
	
	/// Get the x-axis
	constexpr inline UnitVec2 GetXAxis(UnitVec2 rot) noexcept
	{
		return rot;
	}
	
	/// Get the u-axis ("u"??? is that a typo??? Anyway, this is the reverse perpendicular vector of rot as a directional vector)
	constexpr inline UnitVec2 GetYAxis(UnitVec2 rot) noexcept
	{
		return rot.GetRevPerpendicular();
	}

	constexpr inline bool operator == (const UnitVec2 a, const UnitVec2 b) noexcept
	{
		return (a.GetX() == b.GetX()) && (a.GetY() == b.GetY());
	}
	
	constexpr inline bool operator != (const UnitVec2 a, const UnitVec2 b) noexcept
	{
		return (a.GetX() != b.GetX()) || (a.GetY() != b.GetY());
	}

	/// Gets a vector counter-clockwise (reverse-clockwise) perpendicular to the given vector.
	/// @detail This takes a vector of form (x, y) and returns the vector (-y, x).
	/// @param vector Vector to return a counter-clockwise perpendicular equivalent for.
	/// @return A counter-clockwise 90-degree rotation of the given vector.
	/// @sa GetFwdPerpendicular.
	constexpr inline UnitVec2 GetRevPerpendicular(const UnitVec2 vector) noexcept
	{
		return vector.GetRevPerpendicular();
	}
	
	/// Gets a vector clockwise (forward-clockwise) perpendicular to the given vector.
	/// @detail This takes a vector of form (x, y) and returns the vector (y, -x).
	/// @param vector Vector to return a clockwise perpendicular equivalent for.
	/// @return A clockwise 90-degree rotation of the given vector.
	/// @sa GetRevPerpendicular.
	constexpr inline UnitVec2 GetFwdPerpendicular(const UnitVec2 vector) noexcept
	{
		return vector.GetFwdPerpendicular();
	}

	/// Rotates a vector by a given angle.
	constexpr inline UnitVec2 Rotate(const UnitVec2 vector, const UnitVec2& angle) noexcept
	{
		return vector.Rotate(angle);
	}
	
	/// Inverse rotate a vector
	constexpr inline UnitVec2 InverseRotate(const UnitVec2 vector, const UnitVec2& angle) noexcept
	{
		return vector.Rotate(angle.FlipY());
	}

	constexpr inline UnitVec2::data_type GetX(const UnitVec2 value)
	{
		return value.GetX();
	}
	
	constexpr inline UnitVec2::data_type GetY(const UnitVec2 value)
	{
		return value.GetY();
	}
	
}

#endif /* UnitVec2_hpp */
