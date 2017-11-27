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

#include <PlayRho/Common/UnitVec2.hpp>
#include <PlayRho/Common/Math.hpp>

namespace playrho {

UnitVec2::PolarCoord UnitVec2::Get(const Real x, const Real y, const UnitVec2 fallback) noexcept
{
    // Try the faster way first...
    const auto magnitudeSquared = x * x + y * y;
    if (IsNormal(magnitudeSquared))
    {
        const auto magnitude = Sqrt(magnitudeSquared);
        return std::make_pair(UnitVec2{x / magnitude, y / magnitude}, magnitude);
    }
    
    // Failed the faster way, try the more accurate and robust way...
    const auto magnitude = Hypot(x, y);
    if (IsNormal(magnitude))
    {
        return std::make_pair(UnitVec2{x / magnitude, y / magnitude}, magnitude);
    }
    
    // Give up and return the fallback value.
    return std::make_pair(fallback, Real(0));
}

UnitVec2 UnitVec2::Get(const Angle angle) noexcept
{
    return UnitVec2{Cos(angle), Sin(angle)};
}

} // namespace playrho

