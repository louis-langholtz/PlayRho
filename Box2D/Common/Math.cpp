/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include <Box2D/Common/Math.hpp>

using namespace box2d;

Length2D box2d::ComputeCentroid(const Span<const Length2D>& vertices)
{
    assert(vertices.size() >= 3);
    
    auto c = Vec2_zero * Meter * SquareMeter;
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
        
        const auto triangleArea = Area{Cross(e1, e2) / RealNum(2)};
        area += triangleArea;
        
        // Area weighted centroid
        const auto aveP = (p1 + p2 + p3) / RealNum{3};
        c += triangleArea * aveP;
    }
    
    // Centroid
    assert((area > Area{0}) && !almost_zero(area / SquareMeter));
    return c / area;
}

std::vector<Length2D> box2d::GetCircleVertices(const Length radius, unsigned slices,
                                               Angle start, RealNum turns)
{
    std::vector<Length2D> vertices;
    if (slices > 0)
    {
        const auto deltaAngle = (Pi * Radian * RealNum(2) * turns) / slices;
        for (auto i = decltype(slices){0}; i < slices; ++i)
        {
            const auto angleInRadians = RealNum{(start + (i * deltaAngle)) / Radian};
            const auto x = radius * static_cast<RealNum>(std::cos(angleInRadians));
            const auto y = radius * static_cast<RealNum>(std::sin(angleInRadians));
            vertices.push_back(Length2D{x, y});
        }
    }
    return vertices;
}

::std::ostream& box2d::operator<<(::std::ostream& os, const Vec2& value)
{
    return os << "Vec2(" << value.x << "," << value.y << ")";
}

::std::ostream& box2d::operator<<(::std::ostream& os, const UnitVec2& value)
{
    return os << "UnitVec2(" << value.GetX() << "," << value.GetY() << ")";
}

::std::ostream& box2d::operator<<(::std::ostream& os, const Fixed32& value)
{
    return os << static_cast<double>(value);
}

#ifndef _WIN32
::std::ostream& box2d::operator<<(::std::ostream& os, const Fixed64& value)
{
    return os << static_cast<double>(value);
}
#endif
