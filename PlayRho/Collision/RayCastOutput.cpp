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

#include <PlayRho/Collision/RayCastOutput.hpp>
#include <PlayRho/Collision/RayCastInput.hpp>
#include <PlayRho/Collision/AABB.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Shapes/Shape.hpp>
#include <PlayRho/Dynamics/Fixture.hpp>
#include <utility>

using namespace playrho;

RayCastOutput playrho::RayCast(const Length radius, const Length2D location,
                             const RayCastInput& input) noexcept
{
    // Collision Detection in Interactive 3D Environments by Gino van den Bergen
    // From Section 3.1.2
    // x = s + a * r
    // norm(x) = radius
    
    const auto s = input.p1 - location;
    const auto b = GetLengthSquared(s) - Square(radius);
    
    // Solve quadratic equation.
    const auto raySegment = input.p2 - input.p1; // Length2D
    const auto c =  Dot(s, raySegment); // Area
    const auto rr = GetLengthSquared(raySegment); // Area
    const auto sigma = (Square(c) - rr * b) / (SquareMeter * SquareMeter);
    
    // Check for negative discriminant and short segment.
    if ((sigma < Real{0}) || almost_zero(Real{rr / SquareMeter}))
    {
        return RayCastOutput{};
    }
    
    // Find the point of intersection of the line with the circle.
    const auto a = -(c + Sqrt(sigma) * SquareMeter);
    const auto fraction = a / rr;

    // Is the intersection point on the segment?
    if ((fraction >= Real{0}) && (fraction <= input.maxFraction))
    {
        return RayCastOutput{
            GetUnitVector(s + fraction * raySegment, UnitVec2::GetZero()),
            fraction
        };
    }
    
    return RayCastOutput{};
}

RayCastOutput playrho::RayCast(const AABB& aabb, const RayCastInput& input) noexcept
{
    // From Real-time Collision Detection, p179.

    auto tmin = -MaxFloat;
    auto tmax = MaxFloat;
    
    const auto p1 = input.p1;
    const auto pDelta = input.p2 - input.p1;
    
    UnitVec2 normal;
    
    for (auto i = decltype(pDelta.max_size()){0}; i < pDelta.max_size(); ++i)
    {
        const auto p1i = p1[i];
        const auto pdi = pDelta[i];
        const auto lbi = aabb.GetLowerBound()[i];
        const auto ubi = aabb.GetUpperBound()[i];

        if (almost_zero(pdi / Meter))
        {
            // Parallel.
            if ((p1i < lbi) || (ubi < p1i))
            {
                return RayCastOutput{};
            }
        }
        else
        {
            auto t1 = Real{(lbi - p1i) / pdi};
            auto t2 = Real{(ubi - p1i) / pdi};
            
            // Sign of the normal vector.
            auto s = -1;
            
            if (t1 > t2)
            {
                Swap(t1, t2);
                s = 1;
            }
            
            // Push the min up
            if (tmin < t1)
            {
                normal = (i == 0)?
                    ((s < 0)? UnitVec2::GetLeft(): UnitVec2::GetRight()):
                    ((s < 0)? UnitVec2::GetBottom(): UnitVec2::GetTop());
                tmin = t1;
            }
            
            // Pull the max down
            tmax = Min(tmax, t2);
            
            if (tmin > tmax)
            {
                return RayCastOutput{};
            }
        }
    };
    
    // Does the ray start inside the box?
    // Does the ray intersect beyond the max fraction?
    if ((tmin < 0) || (tmin > input.maxFraction))
    {
        return RayCastOutput{};
    }
    
    // Intersection.
    return RayCastOutput{normal, tmin};
}

RayCastOutput playrho::RayCast(const DistanceProxy& proxy, const RayCastInput& input,
                             const Transformation& transform) noexcept
{
    const auto vertexCount = proxy.GetVertexCount();
    assert(vertexCount > 0);

    const auto radius = proxy.GetVertexRadius();
    auto v0 = proxy.GetVertex(0);
    if (vertexCount == 1)
    {
        return ::RayCast(radius, Transform(v0, transform), input);
    }

    // Uses algorithm described at http://stackoverflow.com/a/565282/7410358
    //
    // The SO author gave the algorithm the following credit:
    //   "Intersection of two lines in three-space" by Ronald Goldman,
    //     published in Graphics Gems, page 304.

    // Solve for p + t r = q + u s
    
    // p is input.p1
    // q is the offset vertex
    // s is vertexDelta
    // r is rayDelta
    // t = (q − p) × s / (r × s)
    // u = (q − p) × r / (r × s)

    // Put the ray into the polygon's frame of reference.
    const auto transformedInput = RayCastInput{
        InverseTransform(input.p1, transform),
        InverseTransform(input.p2, transform),
        input.maxFraction
    };
    const auto ray0 = transformedInput.p1;
    const auto ray = transformedInput.p2 - transformedInput.p1; // Ray delta (p2 - p1)
    
    auto minT = std::nextafter(input.maxFraction, Real(2));
    auto normalFound = GetInvalid<UnitVec2>();
    
    for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
    {
        const auto circleResult = ::RayCast(radius, v0, transformedInput);
        if (minT > circleResult.fraction)
        {
            minT = circleResult.fraction;
            normalFound = circleResult.normal;
        }

        const auto v1 = proxy.GetVertex(GetModuloNext(i, vertexCount));
        const auto edge = v1 - v0; // Vertex delta
        const auto ray_cross_edge = Cross(ray, edge);
        
        if (ray_cross_edge != Area{0})
        {
            const auto normal = proxy.GetNormal(i);
            const auto offset = normal * radius;
            const auto v0off = v0 + offset;
            const auto q_sub_p = v0off - ray0;
            
            // t = ((q − p) × s) / (r × s)
            const auto t = Cross(q_sub_p, edge) / ray_cross_edge;
            
            // u = ((q − p) × r) / (r × s)
            const auto u = Cross(q_sub_p, ray) / ray_cross_edge;

            if ((t >= Real(0)) && (t <= Real(1)) &&
                (u >= Real(0)) && (u <= Real(1)))
            {
                // The two lines meet at the point p + t r = q + u s
                if (minT > t)
                {
                    minT = t;
                    normalFound = normal;
                }
            }
            else
            {
                // The two line segments are not parallel but do not intersect.
            }
        }
        else
        {
            // The two lines are parallel, igonred.
        }
        
        v0 = v1;
    }
    
    if (minT <= input.maxFraction)
    {
        return RayCastOutput{Rotate(normalFound, transform.q), minT};
    }
    return RayCastOutput{};
}

RayCastOutput playrho::RayCast(const Shape& shape, ChildCounter childIndex,
                             const RayCastInput& input, const Transformation& transform) noexcept
{
    return RayCast(shape.GetChild(childIndex), input, transform);
}
