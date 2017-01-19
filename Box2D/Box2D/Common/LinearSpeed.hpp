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

#ifndef LinearSpeed_hpp
#define LinearSpeed_hpp

#include <Box2D/Common/Settings.hpp>

namespace box2d
{
	class LinearSpeed
	{
	public:
		using data_type = RealNum;
		
		static LinearSpeed GetFromMetersPerSecond(data_type value)
		{
			return LinearSpeed(value);
		}
		
		LinearSpeed() noexcept = default;
		
		constexpr data_type CvtToMetersPerSecond() const noexcept
		{
			return m_value;
		}
		
	private:
		
		constexpr LinearSpeed(data_type value):
			m_value{value}
		{
			// Intentionally empty.
		}
		
		data_type m_value;
	};
	
	constexpr LinearSpeed operator"" _mps(long double value) noexcept
	{
		return Length::GetFromMetersPerSecond(value);
	}
	
	constexpr LinearSpeed operator"" _mps(unsigned long long int value) noexcept
	{
		return LinearSpeed::GetFromMetersPerSecond(value);
	}
	
} // namespace box2d

#endif /* LinearSpeed_hpp */
