/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <PlayRho/Collision/MassData.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Collision/Shapes/EdgeShape.hpp>
#include <PlayRho/Collision/Shapes/PolygonShape.hpp>
#include <PlayRho/Collision/Shapes/ChainShape.hpp>
#include <PlayRho/Collision/Shapes/DiskShape.hpp>
#include <PlayRho/Collision/Shapes/ChainShape.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <PlayRho/Dynamics/Body.hpp>

using namespace box2d;

MassData box2d::GetMassData(const Length r, const NonNegative<Density> density,
                            const Length2D location)
{
    // Uses parallel axis theorem, perpendicular axis theorem, and the second moment of area.
    // See: https://en.wikipedia.org/wiki/Second_moment_of_area
    //
    // Ixp = Ix + A * dx^2
    // Iyp = Iy + A * dy^2
    // Iz = Ixp + Iyp = Ix + A * dx^2 + Iy + A * dy^2
    // Ix = Pi * r^4 / 4
    // Iy = Pi * r^4 / 4
    // Iz = (Pi * r^4 / 4) + (Pi * r^4 / 4) + (A * dx^2) + (A * dy^2)
    //    = (Pi * r^4 / 2) + (A * (dx^2 + dy^2))
    // A = Pi * r^2
    // Iz = (Pi * r^4 / 2) + (2 * (Pi * r^2) * (dx^2 + dy^2))
    // Iz = Pi * r^2 * ((r^2 / 2) + (dx^2 + dy^2))
    const auto r_squared = r * r;
    const auto area = r_squared * Pi;
    const auto mass = Mass{Density{density} * area};
    const auto Iz = SecondMomentOfArea{area * ((r_squared / Real{2}) + GetLengthSquared(location))};
    const auto I = RotInertia{Iz * Density{density} / SquareRadian};
    return MassData{mass, location, I};
}

MassData box2d::GetMassData(const Length r, const NonNegative<Density> density,
                            const Length2D v0, const Length2D v1)
{
    const auto r_squared = Area{r * r};
    const auto circle_area = r_squared * Pi;
    const auto circle_mass = Density{density} * circle_area;
    const auto d = v1 - v0;
    const auto offset = GetRevPerpendicular(GetUnitVector(d, UnitVec2::GetZero())) * r;
    const auto b = GetLength(d);
    const auto h = r * Real{2};
    const auto rect_mass = Density{density} * b * h;
    const auto totalMass = circle_mass + rect_mass;
    const auto center = (v0 + v1) / Real{2};

    /// Use the fixture's areal mass density times the shape's second moment of area to derive I.
    /// @sa https://en.wikipedia.org/wiki/Second_moment_of_area
    const auto halfCircleArea = circle_area / Real{2};
    const auto halfRSquared = r_squared / Real{2};
    
    const auto vertices = Span<const Length2D>{
        Length2D{v0 + offset},
        Length2D{v0 - offset},
        Length2D{v1 - offset},
        Length2D{v1 + offset}
    };
    assert(vertices.size() == 4);
    const auto I_z = GetPolarMoment(vertices);
    const auto I0 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetLengthSquared(v0))};
    const auto I1 = SecondMomentOfArea{halfCircleArea * (halfRSquared + GetLengthSquared(v1))};
    assert(I0 >= SecondMomentOfArea{0});
    assert(I1 >= SecondMomentOfArea{0});
    assert(I_z >= SecondMomentOfArea{0});
    const auto I = RotInertia{(I0 + I1 + I_z) * Density{density} / SquareRadian};
    return MassData{totalMass, center, I};
}

Area box2d::GetAreaOfCircle(Length radius)
{
    return Area{radius * radius * Pi};
}

Area box2d::GetAreaOfPolygon(Span<const Length2D> vertices)
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
        sum += this_v.x * (next_v.y - last_v.y);
    }
    return sum / Real{2};
}

SecondMomentOfArea box2d::GetPolarMoment(Span<const Length2D> vertices)
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
        const auto fact_b = this_v.x * next_v.y - next_v.x * this_v.y;
        sum_x += [&]() {
            const auto fact_a = Square(this_v.y) + this_v.y * next_v.y + Square(next_v.y);
            return fact_a * fact_b;
        }();
        sum_y += [&]() {
            const auto fact_a = Square(this_v.x) + this_v.x * next_v.x + Square(next_v.x);
            return fact_a * fact_b;
        }();
    }
    const auto secondMomentOfAreaX = SecondMomentOfArea{sum_x};
    const auto secondMomentOfAreaY = SecondMomentOfArea{sum_y};
    return (secondMomentOfAreaX + secondMomentOfAreaY) / Real{12};
}

MassData box2d::GetMassData(const Fixture& f)
{
    return f.GetShape()->GetMassData();
}

MassData box2d::ComputeMassData(const Body& body) noexcept
{
    auto mass = Mass{0};
    auto I = RotInertia{0};
    auto center = Length2D(0, 0);
    for (auto&& f: body.GetFixtures())
    {
        const auto& fixture = GetRef(f);
        if (fixture.GetDensity() > Density{0})
        {
            const auto massData = GetMassData(fixture);
            mass += Mass{massData.mass};
            center += Real{Mass{massData.mass} / Kilogram} * massData.center;
            I += RotInertia{massData.I};
        }
    }
    return MassData{mass, center, I};
}

MassData box2d::GetMassData(const Body& body) noexcept
{
    const auto I = GetLocalInertia(body);
    return MassData{GetMass(body), body.GetLocalCenter(), I};
}
