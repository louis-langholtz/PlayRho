/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Common/Math.hpp>

namespace playrho {

Length2D ComputeCentroid(const Span<const Length2D>& vertices)
{
    assert(vertices.size() >= 3);
    
    auto c = Length2D{} * Area{0};
    auto area = Area{0};
    
    // pRef is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    const auto pRef = Average(vertices);

    for (auto i = decltype(vertices.size()){0}; i < vertices.size(); ++i)
    {
        // Triangle vertices.
        const auto p1 = pRef;
        const auto p2 = vertices[i];
        const auto p3 = vertices[GetModuloNext(i, vertices.size())];
        
        const auto e1 = p2 - p1;
        const auto e2 = p3 - p1;
        
        const auto triangleArea = Area{Cross(e1, e2) / Real(2)};
        area += triangleArea;
        
        // Area weighted centroid
        const auto aveP = (p1 + p2 + p3) / Real{3};
        c += triangleArea * aveP;
    }
    
    // Centroid
    assert((area > Area{0}) && !AlmostZero(area / SquareMeter));
    return c / area;
}

std::vector<Length2D> GetCircleVertices(Length radius, unsigned slices, Angle start, Real turns)
{
    std::vector<Length2D> vertices;
    if (slices > 0)
    {
        const auto integralTurns = static_cast<long int>(turns);
        const auto wholeNum = (turns == Real(integralTurns * Real(1)));
        const auto deltaAngle = (Pi * Radian * Real(2) * turns) / Real(slices);
        auto i = decltype(slices){0};
        while (i < slices)
        {
            const auto angleInRadians = Real{(start + (Real(i) * deltaAngle)) / Radian};
            const auto x = radius * static_cast<Real>(std::cos(angleInRadians));
            const auto y = radius * static_cast<Real>(std::sin(angleInRadians));
            vertices.emplace_back(x, y);
            ++i;
        }
        if (wholeNum)
        {
            // Ensure whole circles come back to original point EXACTLY.
            vertices.push_back(vertices[0]);
        }
        else
        {
            const auto angleInRadians = Real{(start + (Real(i) * deltaAngle)) / Radian};
            const auto x = radius * static_cast<Real>(std::cos(angleInRadians));
            const auto y = radius * static_cast<Real>(std::sin(angleInRadians));
            vertices.emplace_back(x, y);
        }
    }
    return vertices;
}

NonNegative<Area> GetAreaOfCircle(Length radius)
{
    return Area{radius * radius * Pi};
}

NonNegative<Area> GetAreaOfPolygon(Span<const Length2D> vertices)
{
    // Uses the "Shoelace formula".
    // See: https://en.wikipedia.org/wiki/Shoelace_formula
    auto sum = Real(0) * SquareMeter;
    const auto count = vertices.size();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto last_v = vertices[GetModuloPrev(i, count)];
        const auto this_v = vertices[i];
        const auto next_v = vertices[GetModuloNext(i, count)];
        sum += GetX(this_v) * (GetY(next_v) - GetY(last_v));
    }
    
    // Note that using the absolute value isn't necessary for vertices in counter-clockwise
    // ordering; only needed for clockwise ordering.
    return Abs(sum) / Real{2};
}

SecondMomentOfArea GetPolarMoment(Span<const Length2D> vertices)
{
    assert(vertices.size() > 2);
    
    // Use formulas Ix and Iy for second moment of area of any simple polygon and apply
    // the perpendicular axis theorem on these to get the desired answer.
    //
    // See:
    // https://en.wikipedia.org/wiki/Second_moment_of_area#Any_polygon
    // https://en.wikipedia.org/wiki/Second_moment_of_area#Perpendicular_axis_theorem
    auto sum_x = SquareMeter * SquareMeter * Real(0);
    auto sum_y = SquareMeter * SquareMeter * Real(0);
    const auto count = vertices.size();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto this_v = vertices[i];
        const auto next_v = vertices[GetModuloNext(i, count)];
        const auto fact_b = Cross(this_v, next_v);
        sum_x += [&]() {
            const auto fact_a = Square(GetY(this_v)) + GetY(this_v) * GetY(next_v) + Square(GetY(next_v));
            return fact_a * fact_b;
        }();
        sum_y += [&]() {
            const auto fact_a = Square(GetX(this_v)) + GetX(this_v) * GetX(next_v) + Square(GetX(next_v));
            return fact_a * fact_b;
        }();
    }
    const auto secondMomentOfAreaX = SecondMomentOfArea{sum_x};
    const auto secondMomentOfAreaY = SecondMomentOfArea{sum_y};
    return (secondMomentOfAreaX + secondMomentOfAreaY) / Real{12};
}

} // namespace playrho
