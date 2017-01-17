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

#ifndef Length_hpp
#define Length_hpp

#include <Box2D/Common/Settings.hpp>

namespace box2d
{
	class Length
	{
	public:
		using data_type = float_t;

		static Length GetFromMeters(data_type value)
		{
			return Length(value);
		}
		
		Length() noexcept = default;

		constexpr data_type CvtToMeters() const noexcept
		{
			return m_value;
		}

	private:
		
		constexpr Length(data_type value):
			m_value{value}
		{
			// Intentionally empty.
		}

		data_type m_value;
	};
	
	constexpr Length operator"" _m(long double value) noexcept
	{
		return Length::GetFromMeters(value);
	}
	
	constexpr Length operator"" _m(unsigned long long int value) noexcept
	{
		return Length::GetFromMeters(value);
	}

	constexpr inline bool operator== (Length lhs, Length rhs)
	{
		return lhs.CvtToMeters() == rhs.CvtToMeters();
	}
	
	constexpr inline bool operator!= (Length lhs, Length rhs)
	{
		return lhs.CvtToMeters() != rhs.CvtToMeters();
	}
	
	constexpr inline bool operator>= (Length lhs, Length rhs)
	{
		return lhs.CvtToMeters() >= rhs.CvtToMeters();
	}
	
	constexpr inline bool operator> (Length lhs, Length rhs)
	{
		return lhs.CvtToMeters() > rhs.CvtToMeters();
	}
	
	constexpr inline bool operator<= (Length lhs, Length rhs)
	{
		return lhs.CvtToMeters() <= rhs.CvtToMeters();
	}
	
	constexpr inline bool operator< (Length lhs, Length rhs)
	{
		return lhs.CvtToMeters() < rhs.CvtToMeters();
	}
	
	constexpr inline Length operator+ (Length lhs, Length rhs)
	{
		return Length::GetFromMeters(lhs.CvtToMeters() + rhs.CvtToMeters());
	}
	
	constexpr inline Length operator- (Length lhs, Length rhs)
	{
		return Length::GetFromMeters(lhs.CvtToMeters() - rhs.CvtToMeters());
	}
	
	constexpr inline Length operator* (Length::data_type scalar, Length angle)
	{
		return Length::GetFromMeters(angle.CvtToMeters() * scalar);
	}
	
	constexpr inline Length operator* (Length angle, Length::data_type scalar)
	{
		return Length::GetFromMeters(angle.CvtToMeters() * scalar);
	}
	
	constexpr inline Length operator/ (Length angle, Length::data_type scalar)
	{
		return Length::GetFromMeters(angle.CvtToMeters() / scalar);
	}
	
	constexpr inline Length::data_type operator/ (Length lhs, Length rhs)
	{
		return lhs.GetFromMeters() / rhs.CvtToMeters();
	}

}

#endif /* Length_hpp */
