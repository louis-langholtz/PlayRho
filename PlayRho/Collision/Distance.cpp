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

#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/Simplex.hpp>

namespace playrho {

namespace {

    inline bool Find(IndexPair3 pairs, IndexPair key)
    {
        return pairs[0] == key || pairs[1] == key || pairs[2] == key;
    }

    inline SimplexEdge GetSimplexEdge(const DistanceProxy& proxyA,
                                      const Transformation& xfA,
                                      DistanceProxy::size_type idxA,
                                      const DistanceProxy& proxyB,
                                      const Transformation& xfB,
                                      DistanceProxy::size_type idxB)
    {
        const auto wA = Transform(proxyA.GetVertex(idxA), xfA);
        const auto wB = Transform(proxyB.GetVertex(idxB), xfB);
        return SimplexEdge{wA, idxA, wB, idxB};
    }
    
    inline Simplex::Edges GetSimplexEdges(const IndexPair3 indexPairs,
                                          const DistanceProxy& proxyA, const Transformation& xfA,
                                          const DistanceProxy& proxyB, const Transformation& xfB)
    {
        using size_type = std::remove_const<decltype(MaxSimplexEdges)>::type;

        Simplex::Edges simplexEdges;
        const auto count = GetNumIndices(indexPairs);
        switch (count)
        {
            case 3:
                simplexEdges[2] = GetSimplexEdge(proxyA, xfA, indexPairs[2].a,
                                                 proxyB, xfB, indexPairs[2].b);
                // [[fallthrough]]
            case 2:
                simplexEdges[1] = GetSimplexEdge(proxyA, xfA, indexPairs[1].a,
                                                 proxyB, xfB, indexPairs[1].b);
                // [[fallthrough]]
            case 1:
                simplexEdges[0] = GetSimplexEdge(proxyA, xfA, indexPairs[0].a,
                                                 proxyB, xfB, indexPairs[0].b);
                // [[fallthrough]]
        }
        simplexEdges.size(static_cast<size_type>(count));
        return simplexEdges;
    }

}

WitnessPoints GetWitnessPoints(const Simplex& simplex) noexcept
{
    auto pointA = Length2D{};
    auto pointB = Length2D{};

    const auto size = simplex.GetSize();
    for (auto i = decltype(size){0}; i < size; ++i)
    {
        const auto e = simplex.GetSimplexEdge(i);
        const auto c = simplex.GetCoefficient(i);

        pointA += e.GetPointA() * c;
        pointB += e.GetPointB() * c;
    }
#if 0
    // In the 3-simplex case, pointA and pointB are usually equal.
    // XXX: Sometimes in the 3-simplex case, pointA is slightly different than pointB. Why??
    if (size == 3 && pointA != pointB)
    {
        std::cout << "odd: " << pointA << " != " << pointB;
        std::cout << std::endl;
    }
#endif
    return WitnessPoints{pointA, pointB};
}

DistanceOutput Distance(const DistanceProxy& proxyA, const Transformation& transformA,
                        const DistanceProxy& proxyB, const Transformation& transformB,
                        const DistanceConf conf)
{
    assert(proxyA.GetVertexCount() > 0);
    assert(IsValid(transformA.p));
    assert(proxyB.GetVertexCount() > 0);
    assert(IsValid(transformB.p));

    // Initialize the simplex.
    auto simplexEdges = GetSimplexEdges(conf.cache.GetIndices(), proxyA, transformA, proxyB, transformB);

    // Compute the new simplex metric, if it is substantially different than
    // old metric then flush the simplex.
    if (simplexEdges.size() > 1)
    {
        const auto metric1 = conf.cache.GetMetric();
        const auto metric2 = Simplex::CalcMetric(simplexEdges);
        if ((metric2 < (metric1 / 2)) || (metric2 > (metric1 * 2)) || (metric2 < 0) || AlmostZero(metric2))
        {
            simplexEdges.clear();
        }
    }

    if (simplexEdges.size() == 0)
    {
        simplexEdges.push_back(GetSimplexEdge(proxyA, transformA, 0, proxyB, transformB, 0));
    }

    auto simplex = Simplex{};
    auto state = DistanceOutput::HitMaxIters;

#if defined(DO_COMPUTE_CLOSEST_POINT)
    auto distanceSqr1 = MaxFloat;
#endif

    // Main iteration loop.
    auto iter = decltype(conf.maxIterations){0};
    while (iter < conf.maxIterations)
    {
        ++iter;

        // Copy simplex so we can identify duplicates and prevent cycling.
        const auto savedIndices = Simplex::GetIndexPairs(simplexEdges);

        simplex = Simplex::Get(simplexEdges);
        simplexEdges = simplex.GetEdges();

        // If we have max points (3), then the origin is in the corresponding triangle.
        if (simplexEdges.size() == simplexEdges.max_size())
        {
            state = DistanceOutput::MaxPoints;
            break;
        }

#if defined(DO_COMPUTE_CLOSEST_POINT)
        // Compute closest point.
        const auto p = GetClosestPoint(simplexEdges);
        const auto distanceSqr2 = GetLengthSquared(p);

        // Ensure progress
        if (distanceSqr2 >= distanceSqr1)
        {
            //break;
        }
        distanceSqr1 = distanceSqr2;
#endif
        // Get search direction.
        const auto d = Simplex::CalcSearchDirection(simplexEdges);
        assert(IsValid(d));

        // Ensure the search direction is numerically fit.
        if (AlmostZero(StripUnit(GetLengthSquared(d))))
        {
            state = DistanceOutput::UnfitSearchDir;

            // The origin is probably contained by a line segment
            // or triangle. Thus the shapes are overlapped.

            // We can't return zero here even though there may be overlap.
            // In case the simplex is a point, segment, or triangle it is difficult
            // to determine if the origin is contained in the CSO or very close to it.
            break;
        }

        // Compute a tentative new simplex edge using support points.
        const auto indexA = GetSupportIndex(proxyA, GetVec2(InverseRotate(-d, transformA.q)));
        const auto indexB = GetSupportIndex(proxyB, GetVec2(InverseRotate(d, transformB.q)));

        // Check for duplicate support points. This is the main termination criteria.
        // If there's a duplicate support point, code must exit loop to avoid cycling.
        if (Find(savedIndices, IndexPair{indexA, indexB}))
        {
            state = DistanceOutput::DuplicateIndexPair;
            break;
        }

        // New edge is ok and needed.
        simplexEdges.push_back(GetSimplexEdge(proxyA, transformA, indexA, proxyB, transformB, indexB));
    }

    // Note: simplexEdges is same here as simplex.GetSimplexEdges().
    // GetWitnessPoints(simplex), iter, Simplex::GetCache(simplexEdges)
    return DistanceOutput{simplex, iter, state};
}

Area TestOverlap(const DistanceProxy& proxyA, const Transformation& xfA,
                 const DistanceProxy& proxyB, const Transformation& xfB,
                 const DistanceConf conf)
{
    const auto distanceInfo = Distance(proxyA, xfA, proxyB, xfB, conf);
    assert(distanceInfo.state != DistanceOutput::Unknown && distanceInfo.state != DistanceOutput::HitMaxIters);
    
    const auto witnessPoints = GetWitnessPoints(distanceInfo.simplex);
    const auto distanceSquared = GetLengthSquared(witnessPoints.a - witnessPoints.b);
    const auto totalRadiusSquared = Square(proxyA.GetVertexRadius() + proxyB.GetVertexRadius());
    return totalRadiusSquared - distanceSquared;
}

} // namespace playrho
