/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/UnitVec2.hpp>
#include <Box2D/Common/Math.hpp>

using namespace box2d;

UnitVec2 UnitVec2::Get(const RealNum x, const RealNum y, RealNum& magnitude, const UnitVec2 fallback)
{
    if (IsValid(x) && IsValid(y))
    {
        // XXX perhaps this should use std::hypot() instead like so:
        //    magnitude = std::hypot(x, y);
        magnitude = Sqrt(Square(x) + Square(y));
        return (!almost_zero(magnitude))? UnitVec2{x / magnitude, y / magnitude}: fallback;
    }
    return UnitVec2{};
}

UnitVec2::UnitVec2(const Angle angle) noexcept:
    m_x{std::cos(angle / Radian)}, m_y{std::sin(angle / Radian)}
{
    // Intentionally empty.
}
