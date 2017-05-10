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

#include <Box2D/Collision/DistanceProxy.hpp>
#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/Shapes/ChainShape.hpp>
#include <Box2D/Collision/Shapes/PolygonShape.hpp>

namespace box2d {
    
DistanceProxy::size_type GetSupportIndex(const DistanceProxy& proxy, const Length2D d) noexcept
{
    auto index = DistanceProxy::InvalidIndex; ///< Index of vertex that when dotted with d has the max value.
    auto maxValue = -MaxFloat * SquareMeter; ///< Max dot value.
    const auto count = proxy.GetVertexCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto value = Dot(proxy.GetVertex(i), d);
        if (maxValue < value)
        {
            maxValue = value;
            index = i;
        }
    }
    return index;
}

DistanceProxy GetDistanceProxy(const Shape& shape, child_count_t index)
{
    return shape.GetChild(index);
}

size_t FindLowestRightMostVertex(Span<const Length2D> vertices)
{
    const auto size = vertices.size();
    if (size > 0)
    {
        auto i0 = decltype(size){0};
        auto max_x = vertices[0].x;
        for (auto i = decltype(vertices.size()){1}; i < vertices.size(); ++i)
        {
            const auto x = vertices[i].x;
            if ((max_x < x) || ((max_x == x) && (vertices[i].y < vertices[i0].y)))
            {
                max_x = x;
                i0 = i;
            }
        }
        return i0;
    }
    return static_cast<size_t>(-1);
}

std::vector<Length2D> GetConvexHullAsVector(Span<const Length2D> vertices)
{
    std::vector<Length2D> result;
    
    // Create the convex hull using the Gift wrapping algorithm
    // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
    
    const auto index0 = FindLowestRightMostVertex(vertices);
    if (index0 != static_cast<decltype(index0)>(-1))
    {
        const auto size = vertices.size();
        auto hull = std::vector<decltype(vertices.size())>();
        
        auto ih = index0;
        for (;;)
        {
            hull.push_back(ih);
            
            auto ie = decltype(size){0};
            for (auto j = decltype(size){1}; j < size; ++j)
            {
                if (ie == ih)
                {
                    ie = j;
                    continue;
                }
                
                const auto r = vertices[ie] - vertices[ih];
                const auto v = vertices[j] - vertices[ih];
                const auto c = Cross(r, v);
                if ((c < Area{0}) || ((c == Area{0}) && (GetLengthSquared(v) > GetLengthSquared(r))))
                {
                    ie = j;
                }
            }
            
            ih = ie;
            if (ie == index0)
            {
                break;
            }
        }
        
        const auto count = hull.size();
        for (auto i = decltype(count){0}; i < count; ++i)
        {
            result.emplace_back(vertices[hull[i]]);
        }
    }
    
    return result;
}

} // namespace box2d
