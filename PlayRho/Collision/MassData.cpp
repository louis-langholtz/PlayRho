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

using namespace playrho;

MassData playrho::GetMassData(const Length r, const NonNegative<Density> density,
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

MassData playrho::GetMassData(const Length r, const NonNegative<Density> density,
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

MassData playrho::GetMassData(const Length vertexRadius, const NonNegative<Density> density,
                              Span<const Length2D> vertices)
{
    // See: https://en.wikipedia.org/wiki/Centroid#Centroid_of_polygon
    
    // Polygon mass, centroid, and inertia.
    // Let rho be the polygon density in mass per unit area.
    // Then:
    // mass = rho * int(dA)
    // centroid.x = (1/mass) * rho * int(x * dA)
    // centroid.y = (1/mass) * rho * int(y * dA)
    // I = rho * int((x*x + y*y) * dA)
    //
    // We can compute these integrals by summing all the integrals
    // for each triangle of the polygon. To evaluate the integral
    // for a single triangle, we make a change of variables to
    // the (u,v) coordinates of the triangle:
    // x = x0 + e1x * u + e2x * v
    // y = y0 + e1y * u + e2y * v
    // where 0 <= u && 0 <= v && u + v <= 1.
    //
    // We integrate u from [0,1-v] and then v from [0,1].
    // We also need to use the Jacobian of the transformation:
    // D = cross(e1, e2)
    //
    // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
    //
    // The rest of the derivation is handled by computer algebra.
    
    const auto count = vertices.size();
    switch (count)
    {
        case 0:
            return MassData{
                Mass{Kilogram * GetInvalid<Real>()},
                GetInvalid<Length2D>(),
                RotInertia{SquareMeter * Kilogram * GetInvalid<Real>() / SquareRadian}
            };
        case 1:
            return ::GetMassData(vertexRadius, density, vertices[0]);
        case 2:
            return ::GetMassData(vertexRadius, density, vertices[0], vertices[1]);;
        default:
            break;
    }
    
    auto center = Length2D(0, 0);
    auto area = Area{0};
    auto I = SecondMomentOfArea{0};
    
    // s is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    // This code puts the reference point inside the polygon.
    const auto s = Average(vertices);
    
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        // Triangle vertices.
        const auto e1 = vertices[i] - s;
        const auto e2 = vertices[GetModuloNext(i, count)] - s;
        
        const auto D = Cross(e1, e2);
        
        const auto triangleArea = D / Real{2};
        area += triangleArea;
        
        // Area weighted centroid
        center += StripUnit(triangleArea) * (e1 + e2) / Real{3};
        
        const auto intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
        const auto inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
        
        const auto triangleI = D * (intx2 + inty2) / Real{3 * 4};
        I += triangleI;
    }
    
    // Total mass
    const auto mass = Mass{density * area};
    
    // Center of mass
    assert((area > Area{0}) && !almost_zero(StripUnit(area)));
    center /= StripUnit(area);
    const auto massDataCenter = center + s;
    
    // Inertia tensor relative to the local origin (point s).
    // Shift to center of mass then to original body origin.
    const auto massCenterOffset = GetLengthSquared(massDataCenter);
    const auto centerOffset = GetLengthSquared(center);
    const auto intertialLever = massCenterOffset - centerOffset;
    const auto massDataI = RotInertia{((density * I) + (mass * intertialLever)) / SquareRadian};
    
    return MassData{mass, massDataCenter, massDataI};
}

Area playrho::GetAreaOfCircle(Length radius)
{
    return Area{radius * radius * Pi};
}

Area playrho::GetAreaOfPolygon(Span<const Length2D> vertices)
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

SecondMomentOfArea playrho::GetPolarMoment(Span<const Length2D> vertices)
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

MassData playrho::GetMassData(const Fixture& f)
{
    return f.GetShape()->GetMassData();
}

MassData playrho::ComputeMassData(const Body& body) noexcept
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

MassData playrho::GetMassData(const Body& body) noexcept
{
    const auto I = GetLocalInertia(body);
    return MassData{GetMass(body), body.GetLocalCenter(), I};
}
