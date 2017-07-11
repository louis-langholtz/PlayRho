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

#include <Box2D/Collision/ShapeSeparation.hpp>
#include <Box2D/Collision/DistanceProxy.hpp>

using namespace box2d;

IndexPairSeparation box2d::GetMaxSeparation(const DistanceProxy& proxy1, const Transformation xf1,
                                            const DistanceProxy& proxy2, const Transformation xf2,
                                            Length stop)
{
    // Find the max separation between proxy1 and proxy2 using edge normals from proxy1.
    
    auto indexPairSep = IndexPairSeparation{
        -MaxFloat * Meter, IndexPairSeparation::InvalidIndex, IndexPairSeparation::InvalidIndex
    };
    
    const auto xf = MulT(xf2, xf1);
    const auto count1 = proxy1.GetVertexCount();
    const auto count2 = proxy2.GetVertexCount();

    for (auto i = decltype(count1){0}; i < count1; ++i)
    {
        // Get proxy1 normal and vertex relative to proxy2.
        const auto normal = Rotate(proxy1.GetNormal(i), xf.q);
        const auto offset = Transform(proxy1.GetVertex(i), xf);

        // Search for the vector that's most anti-parallel to the normal.
        // See: https://en.wikipedia.org/wiki/Antiparallel_(mathematics)#Antiparallel_vectors
        auto ap = IndexSeparation{};
        for (auto j = decltype(count2){0}; j < count2; ++j)
        {
            // Get distance from offset to proxy2.GetVertex(j) in direction of normal.
            const auto s = Dot(normal, proxy2.GetVertex(j) - offset);
            if (ap.separation > s)
            {
                ap.separation = s;
                ap.index = static_cast<IndexSeparation::index_type>(j);
            }
        }

        if (ap.separation > stop)
        {
            return IndexPairSeparation{
                ap.separation, static_cast<IndexSeparation::index_type>(i), ap.index
            };
        }
        if (indexPairSep.separation < ap.separation)
        {
            indexPairSep.separation = ap.separation;
            indexPairSep.index1 = static_cast<IndexSeparation::index_type>(i);
            indexPairSep.index2 = ap.index;
        }
    }
    return indexPairSep;
}

IndexPairSeparation box2d::GetMaxSeparation(const DistanceProxy& proxy1,
                                            const DistanceProxy& proxy2,
                                            Length stop)
{
    // Find the max separation between proxy1 and proxy2 using edge normals from proxy1.
    
    auto indexPairSep = IndexPairSeparation{
        -MaxFloat * Meter, IndexPairSeparation::InvalidIndex, IndexPairSeparation::InvalidIndex
    };
    
    const auto count1 = proxy1.GetVertexCount();
    const auto count2 = proxy2.GetVertexCount();
    
    for (auto i = decltype(count1){0}; i < count1; ++i)
    {
        // Get proxy1 normal and vertex relative to proxy2.
        const auto normal = proxy1.GetNormal(i);
        const auto offset = proxy1.GetVertex(i);
        
        // Search for the vector that's most anti-parallel to the normal.
        // See: https://en.wikipedia.org/wiki/Antiparallel_(mathematics)#Antiparallel_vectors
        auto ap = IndexSeparation{};
        for (auto j = decltype(count2){0}; j < count2; ++j)
        {
            // Get distance from offset to proxy2.GetVertex(j) in direction of normal.
            const auto s = Dot(normal, proxy2.GetVertex(j) - offset);
            if (ap.separation > s)
            {
                ap.separation = s;
                ap.index = static_cast<IndexSeparation::index_type>(j);
            }
        }
        
        if (ap.separation > stop)
        {
            return IndexPairSeparation{
                ap.separation, static_cast<IndexSeparation::index_type>(i), ap.index
            };
        }
        if (indexPairSep.separation < ap.separation)
        {
            indexPairSep.separation = ap.separation;
            indexPairSep.index1 = static_cast<IndexSeparation::index_type>(i);
            indexPairSep.index2 = ap.index;
        }
    }
    return indexPairSep;
}
