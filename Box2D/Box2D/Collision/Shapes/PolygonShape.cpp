/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/PolygonShape.hpp>
#include <Box2D/Collision/RayCastInput.hpp>
#include <Box2D/Common/VertexSet.hpp>

using namespace box2d;

PolygonShape::PolygonShape(Length hx, Length hy, const Conf& conf) noexcept:
    Shape{conf}
{
    SetAsBox(hx, hy);
}

PolygonShape::PolygonShape(Span<const Length2D> points, const Conf& conf) noexcept:
    Shape{conf}
{
    Set(points);
}

MassData PolygonShape::GetMassData() const noexcept
{
    // See: https://en.wikipedia.org/wiki/Centroid#Centroid_of_polygon
    
    assert(GetDensity() >= Density{0});
    
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
    
    const auto count = GetVertexCount();
    switch (count)
    {
        case 0:
            return MassData{
                Mass{Kilogram * GetInvalid<RealNum>()},
                GetInvalid<Length2D>(),
                RotInertia{SquareMeter * Kilogram * GetInvalid<RealNum>() / SquareRadian}
            };
        case 1:
            return ::GetMassData(GetVertexRadius(), GetDensity(), GetVertex(0));
        case 2:
            return ::GetMassData(GetVertexRadius(), GetDensity(), GetVertex(0), GetVertex(1));;
        default:
            break;
    }
    
    auto center = Vec2_zero * Meter;
    auto area = Area{0};
    auto I = SecondMomentOfArea{0};
    
    // s is the reference point for forming triangles.
    // It's location doesn't change the result (except for rounding error).
    // This code puts the reference point inside the polygon.
    const auto s = Average(GetVertices());
    
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        // Triangle vertices.
        const auto e1 = GetVertex(i) - s;
        const auto e2 = GetVertex(GetModuloNext(i, count)) - s;
        
        const auto D = Cross(e1, e2);
        
        const auto triangleArea = D / RealNum{2};
        area += triangleArea;
        
        // Area weighted centroid
        center += StripUnit(triangleArea) * (e1 + e2) / RealNum{3};
        
        const auto intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
        const auto inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
        
        const auto triangleI = D * (intx2 + inty2) / RealNum{3 * 4};
        I += triangleI;
    }
    
    // Total mass
    const auto mass = Mass{GetDensity() * area};
    
    // Center of mass
    assert((area > Area{0}) && !almost_zero(StripUnit(area)));
    center /= StripUnit(area);
    const auto massDataCenter = center + s;
    
    // Inertia tensor relative to the local origin (point s).
    // Shift to center of mass then to original body origin.
    const auto massCenterOffset = GetLengthSquared(massDataCenter);
    const auto centerOffset = GetLengthSquared(center);
    const auto intertialLever = massCenterOffset - centerOffset;
    const auto massDataI = RotInertia{((GetDensity() * I) + (mass * intertialLever)) / SquareRadian};
    
    return MassData{mass, massDataCenter, massDataI};
}

void PolygonShape::SetAsBox(Length hx, Length hy) noexcept
{
    m_centroid = Vec2_zero * Meter;

    // vertices must be counter-clockwise

    const auto btm_rgt = Length2D{+hx, -hy};
    const auto top_rgt = Length2D{ hx,  hy};
    const auto top_lft = Length2D{-hx, +hy};
    const auto btm_lft = Length2D{-hx, -hy};
    
    m_vertices.clear();
    m_vertices.emplace_back(btm_rgt);
    m_vertices.emplace_back(top_rgt);
    m_vertices.emplace_back(top_lft);
    m_vertices.emplace_back(btm_lft);

    m_normals.clear();
    m_normals.emplace_back(UnitVec2::GetRight());
    m_normals.emplace_back(UnitVec2::GetTop());
    m_normals.emplace_back(UnitVec2::GetLeft());
    m_normals.emplace_back(UnitVec2::GetBottom());
}

void box2d::SetAsBox(PolygonShape& shape, Length hx, Length hy, const Length2D center, Angle angle) noexcept
{
    shape.SetAsBox(hx, hy);
    shape.Transform(Transformation{center, UnitVec2{angle}});
}

void PolygonShape::Transform(box2d::Transformation xf) noexcept
{
    for (auto i = decltype(GetVertexCount()){0}; i < GetVertexCount(); ++i)
    {
        m_vertices[i] = box2d::Transform(m_vertices[i], xf);
        m_normals[i] = Rotate(m_normals[i], xf.q);
    }
    m_centroid = box2d::Transform(m_centroid, xf);
}

void PolygonShape::Set(Span<const Length2D> points) noexcept
{
    // Perform welding and copy vertices into local buffer.
    auto point_set = VertexSet(Square(DefaultLinearSlop));
    for (auto&& p: points)
    {
        point_set.add(p);
    }
    Set(point_set);
}

void PolygonShape::Set(const VertexSet& point_set) noexcept
{
#ifndef NDEBUG
    const auto point_set_size = point_set.size();
    assert(point_set_size > 0 && point_set_size < std::numeric_limits<vertex_count_t>::max());
#endif

    m_vertices = GetConvexHullAsVector(point_set);
    assert(m_vertices.size() > 0 && m_vertices.size() < std::numeric_limits<vertex_count_t>::max());
    
    const auto count = static_cast<vertex_count_t>(m_vertices.size());

    m_normals.clear();
    if (count > 1)
    {
        // Compute normals.
        for (auto i = decltype(count){0}; i < count; ++i)
        {
            const auto edge = GetEdge(*this, i);
            m_normals.emplace_back(GetUnitVector(GetFwdPerpendicular(edge)));
        }
    }
    else if (count == 1)
    {
        m_normals.emplace_back(UnitVec2{});
    }

    // Compute the polygon centroid.
    switch (count)
    {
        case 0:
            m_centroid = GetInvalid<Length2D>();
            break;
        case 1:
            m_centroid = m_vertices[0];
            break;
        case 2:
            m_centroid = (m_vertices[0] + m_vertices[1]) / RealNum{2};
            break;
        default:
            m_centroid = ComputeCentroid(GetVertices());
            break;
    }
}

bool PolygonShape::TestPoint(const Transformation& xf, const Length2D p) const noexcept
{
    const auto dp = p - xf.p;
    const auto pLocal = InverseRotate(dp, xf.q);
    const auto vr = GetVertexRadius();
    const auto count = GetVertexCount();
    
    if (count == 1)
    {
        const auto v0 = GetVertex(0);
        const auto center = xf.p + Rotate(v0, xf.q);
        const auto delta = p - center;
        return GetLengthSquared(delta) <= Square(vr);
    }
    
    auto maxDot = -MaxFloat * Meter;
    auto maxIdx = PolygonShape::InvalidVertex;
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto vi = GetVertex(i);
        const auto delta = pLocal - vi;
        const auto dot = Dot(GetNormal(i), delta);
        if (dot > vr)
        {
            return false;
        }
        if (maxDot < dot)
        {
            maxDot = dot;
            maxIdx = i;
        }
    }
    
    const auto v0 = GetVertex(maxIdx);
    const auto v1 = GetVertex(GetModuloNext(maxIdx, count));
    const auto edge = v1 - v0;
    const auto delta0 = v0 - pLocal;
    const auto d0 = Dot(edge, delta0);
    if (d0 >= Area{0})
    {
        // point is nearest v0 and not within edge
        return GetLengthSquared(delta0) <= Square(vr);
    }
    const auto delta1 = pLocal - v1;
    const auto d1 = Dot(edge, delta1);
    if (d1 >= Area{0})
    {
        // point is nearest v1 and not within edge
        return GetLengthSquared(delta1) <= Square(vr);
    }
    return true;
}

Length2D box2d::GetEdge(const PolygonShape& shape, PolygonShape::vertex_count_t index)
{
    assert(shape.GetVertexCount() > 1);

    const auto i0 = index;
    const auto i1 = GetModuloNext(index, shape.GetVertexCount());
    return shape.GetVertex(i1) - shape.GetVertex(i0);
}

bool box2d::Validate(const PolygonShape& shape)
{
    const auto count = shape.GetVertexCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto i1 = i;
        const auto i2 = GetModuloNext(i1, count);
        const auto p = shape.GetVertex(i1);
        const auto e = shape.GetVertex(i2) - p;
        
        for (auto j = decltype(count){0}; j < count; ++j)
        {
            if ((j == i1) || (j == i2))
            {
                continue;
            }
            
            const auto v = shape.GetVertex(j) - p;
            const auto c = Cross(e, v);
            if (c < Area{0})
            {
                return false;
            }
        }
    }
    
    return true;
}

