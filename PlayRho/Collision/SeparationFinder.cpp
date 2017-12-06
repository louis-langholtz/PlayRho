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

#include <PlayRho/Collision/SeparationFinder.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>

namespace playrho {

SeparationFinder SeparationFinder::Get(IndexPair3 indices,
                                       const DistanceProxy& proxyA, const Transformation& xfA,
                                       const DistanceProxy& proxyB, const Transformation& xfB)
{
    assert(GetNumIndices(indices) > 0);
    assert(proxyA.GetVertexCount() > 0);
    assert(proxyB.GetVertexCount() > 0);
    
    const auto numIndices = GetNumIndices(indices);
    const auto type = (numIndices == 1)? e_points: ((indices[0].first == indices[1].first)? e_faceB: e_faceA);
    
    switch (type)
    {
        case e_points:
        {
            const auto ip0 = indices[0];
            const auto localPointA = proxyA.GetVertex(ip0.first);
            const auto localPointB = proxyB.GetVertex(ip0.second);
            const auto pointA = Transform(localPointA, xfA);
            const auto pointB = Transform(localPointB, xfB);
            const auto axis = GetUnitVector(pointB - pointA, UnitVec2::GetZero());
            return SeparationFinder{proxyA, proxyB, axis, GetInvalid<Length2>(), type};
        }
        case e_faceB:
        {
            const auto ip0 = indices[0];
            const auto ip1 = indices[1];
            
            // Two points on B and one on A.
            const auto localPointB1 = proxyB.GetVertex(ip0.second);
            const auto localPointB2 = proxyB.GetVertex(ip1.second);
            const auto lpDelta = localPointB2 - localPointB1;
            
            const auto axis = GetUnitVector(GetFwdPerpendicular(lpDelta),
                                            UnitVec2::GetZero());
            const auto normal = Rotate(axis, xfB.q);
            
            const auto localPoint = (localPointB1 + localPointB2) / Real(2);
            const auto pointB = Transform(localPoint, xfB);
            
            const auto localPointA = proxyA.GetVertex(ip0.first);
            const auto pointA = Transform(localPointA, xfA);
            
            const auto deltaPoint = pointA - pointB;
            return SeparationFinder{
                proxyA, proxyB,
                (Dot(deltaPoint, normal) < 0_m)? -axis: axis,
                localPoint, type
            };
        }
        case e_faceA:
        {
            const auto ip0 = indices[0];
            const auto ip1 = indices[1];
            
            // Two points on A and one or two points on B.
            const auto localPointA1 = proxyA.GetVertex(ip0.first);
            const auto localPointA2 = proxyA.GetVertex(ip1.first);
            const auto delta = localPointA2 - localPointA1;
            
            const auto axis = GetUnitVector(GetFwdPerpendicular(delta), UnitVec2::GetZero());
            const auto normal = Rotate(axis, xfA.q);
            
            const auto localPoint = (localPointA1 + localPointA2) / Real(2);
            const auto pointA = Transform(localPoint, xfA);
            
            const auto localPointB = proxyB.GetVertex(ip0.second);
            const auto pointB = Transform(localPointB, xfB);
            
            const auto deltaPoint = pointB - pointA;
            return SeparationFinder{
                proxyA, proxyB,
                (Dot(deltaPoint, normal) < 0_m)? -axis: axis,
                localPoint, type
            };
        }
        default: break;
    }

    // Should never be reached
    return SeparationFinder{proxyA, proxyB, UnitVec2{}, GetInvalid<Length2>(), type};
}

IndexPairDistance
SeparationFinder::FindMinSeparationForPoints(const Transformation& xfA,
                                             const Transformation& xfB) const
{
    const auto dirA = InverseRotate(+m_axis, xfA.q);
    const auto dirB = InverseRotate(-m_axis, xfB.q);
    const auto indexA = GetSupportIndex(m_proxyA, GetVec2(dirA));
    const auto indexB = GetSupportIndex(m_proxyB, GetVec2(dirB));
    const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
    const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
    const auto delta = pointB - pointA;
    return IndexPairDistance{Dot(delta, m_axis), IndexPair{indexA, indexB}};
}

IndexPairDistance
SeparationFinder::FindMinSeparationForFaceA(const Transformation& xfA,
                                            const Transformation& xfB) const
{
    const auto normal = Rotate(m_axis, xfA.q);
    const auto indexA = InvalidVertex;
    const auto pointA = Transform(m_localPoint, xfA);
    const auto dir = InverseRotate(-normal, xfB.q);
    const auto indexB = GetSupportIndex(m_proxyB, GetVec2(dir));
    const auto pointB = Transform(m_proxyB.GetVertex(indexB), xfB);
    const auto delta = pointB - pointA;
    return IndexPairDistance{Dot(delta, normal), IndexPair{indexA, indexB}};
}

IndexPairDistance
SeparationFinder::FindMinSeparationForFaceB(const Transformation& xfA,
                                            const Transformation& xfB) const
{
    const auto normal = Rotate(m_axis, xfB.q);
    const auto dir = InverseRotate(-normal, xfA.q);
    const auto indexA = GetSupportIndex(m_proxyA, GetVec2(dir));
    const auto pointA = Transform(m_proxyA.GetVertex(indexA), xfA);
    const auto indexB = InvalidVertex;
    const auto pointB = Transform(m_localPoint, xfB);
    const auto delta = pointA - pointB;
    return IndexPairDistance{Dot(delta, normal), IndexPair{indexA, indexB}};
}

Length SeparationFinder::EvaluateForPoints(const Transformation& xfA, const Transformation& xfB,
                                           IndexPair indexPair) const
{
    const auto pointA = Transform(m_proxyA.GetVertex(indexPair.first), xfA);
    const auto pointB = Transform(m_proxyB.GetVertex(indexPair.second), xfB);
    const auto delta = pointB - pointA;
    return Dot(delta, m_axis);
}

Length SeparationFinder::EvaluateForFaceA(const Transformation& xfA, const Transformation& xfB,
                                          IndexPair indexPair) const
{
    const auto normal = Rotate(m_axis, xfA.q);
    const auto pointA = Transform(m_localPoint, xfA);
    const auto pointB = Transform(m_proxyB.GetVertex(indexPair.second), xfB);
    const auto delta = pointB - pointA;
    return Dot(delta, normal);
}

Length SeparationFinder::EvaluateForFaceB(const Transformation& xfA, const Transformation& xfB,
                                          IndexPair indexPair) const
{
    const auto normal = Rotate(m_axis, xfB.q);
    const auto pointB = Transform(m_localPoint, xfB);
    const auto pointA = Transform(m_proxyA.GetVertex(indexPair.first), xfA);
    const auto delta = pointA - pointB;
    return Dot(delta, normal);
}

} // namespace playrho
