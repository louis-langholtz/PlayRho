/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/UnitVec2.hpp>
#include <Box2D/Common/Math.hpp>

using namespace box2d;

UnitVec2::UnitVec2(const Vec2& value, UnitVec2 fallback) noexcept
{
	if (IsValid(value))
	{
		// XXX perhaps this should use std::hypot() instead like so:
		//    const auto length = std::hypot(value.x, value.y);

		const auto lengthSquared = GetLengthSquared(value);
		if (lengthSquared > 0)
		{
			const auto unitized = value / Sqrt(lengthSquared);
			m_x = unitized.x;
			m_y = unitized.y;
			return;
		}
		m_x = fallback.GetX();
		m_y = fallback.GetY();
		return;
	}
	m_x = GetInvalid<data_type>();
	m_y = GetInvalid<data_type>();
}

UnitVec2::UnitVec2(const Angle& angle) noexcept:
	m_x{std::cos(angle / Radian)}, m_y{std::sin(angle / Radian)}
{
	// Intentionally empty.
}

UnitVec2 box2d::GetUnitVector(const Vec2& value, UnitVec2 fallback)
{
	return UnitVec2{value, fallback};
}
