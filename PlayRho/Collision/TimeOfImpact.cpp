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

#include <PlayRho/Collision/TimeOfImpact.hpp>
#include <PlayRho/Collision/Distance.hpp>
#include <PlayRho/Collision/DistanceProxy.hpp>
#include <PlayRho/Collision/SeparationFinder.hpp>

namespace playrho {

namespace {
        
inline DistanceConf GetDistanceConf(const ToiConf& conf)
{
    DistanceConf distanceConf;
    distanceConf.maxIterations = conf.maxDistIters;
    return distanceConf;
}

} // anonymous namespace

TOIOutput GetToiViaSat(const DistanceProxy& proxyA, const Sweep& sweepA,
                       const DistanceProxy& proxyB, const Sweep& sweepB,
                       const ToiConf conf)
{
    assert(sweepA.GetAlpha0() == sweepB.GetAlpha0());

    // CCD via the local separating axis method. This seeks progression
    // by computing the largest time at which separation is maintained.
    
    auto stats = TOIOutput::Stats{};

    assert(conf.tMax >= 0 && conf.tMax <=1);
    assert(conf.tolerance > Length{0});

    const auto totalRadius = proxyA.GetVertexRadius() + proxyB.GetVertexRadius();
    assert(conf.targetDepth < totalRadius);
    assert(conf.targetDepth >= conf.tolerance);
    
    const auto target = totalRadius - conf.targetDepth;
    assert(target != totalRadius);
    assert(target > conf.tolerance);
    
    const auto maxTarget = target + conf.tolerance;
    assert(maxTarget != target);
    assert(maxTarget <= totalRadius);
    
    const auto minTarget = target - conf.tolerance;
    assert(minTarget != target);
    assert(minTarget < maxTarget);
    assert(minTarget > Length{0} && !almost_zero(minTarget / Meter));
    
    const auto maxTargetSquared = Square(maxTarget);

    auto t1 = Real{0}; // Will be set to value of t2
    auto t1xfA = GetTransformation(sweepA, t1);
    auto t1xfB = GetTransformation(sweepB, t1);

    // Prepare input for distance query.
    auto distanceConf = GetDistanceConf(conf);

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    while (stats.toi_iters < conf.maxToiIters)
    {
        // Get information on the distance between shapes. We can also use the results
        // to get a separating axis.
        const auto distanceInfo = Distance(proxyA, t1xfA, proxyB, t1xfB, distanceConf);
        assert(distanceInfo.state != DistanceOutput::Unknown && distanceInfo.state != DistanceOutput::HitMaxIters);

        distanceConf.cache = Simplex::GetCache(distanceInfo.simplex.GetEdges());

        ++stats.toi_iters;
        stats.sum_dist_iters += distanceInfo.iterations;
        stats.max_dist_iters = std::max(stats.max_dist_iters, distanceInfo.iterations);

        const auto witnessPoints = GetWitnessPoints(distanceInfo.simplex);
        const auto dwp = witnessPoints.a - witnessPoints.b;

        // Get the real distance squared between shapes at the time of t1.
        const auto distanceSquared = GetLengthSquared(dwp);
        
        // If the shapes aren't separated, give up on continuous collision.
        if (distanceSquared <= Area{0}) // Failure!
        {
            return TOIOutput{TOIOutput::e_overlapped, 0, stats};
        }

        if (distanceSquared <= maxTargetSquared) // Victory!
        {
            return TOIOutput{TOIOutput::e_touching, t1, stats};
        }

        // From here on, the real distance squared at time t1 is > than maxTargetSquared

        // Initialize the separating axis.
        const auto fcn = SeparationFinder::Get(distanceConf.cache.GetIndices(), proxyA, t1xfA, proxyB, t1xfB);

        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        auto t2 = conf.tMax; // t2 goes to values between t1 and t2.
        auto t2xfA = GetTransformation(sweepA, t2);
        auto t2xfB = GetTransformation(sweepB, t2);

        auto pbIter = decltype(MaxShapeVertices){0};
        for (; pbIter < MaxShapeVertices; ++pbIter)
        {
            // Find the deepest point at t2. Store the witness point indices.
            const auto t2MinSeparation = fcn.FindMinSeparation(t2xfA, t2xfB);

            // Is the final configuration separated?
            if (t2MinSeparation.distance > maxTarget)
            {
                // Victory! No collision occurs within time span.
                assert(t2 == conf.tMax);
                // Formerly this used tMax as in...
                // return TOIOutput{TOIOutput::e_separated, tMax};
                // t2 seems more appropriate however given s2 was derived from it.
                // Meanwhile t2 always seems equal to input.tMax at this point.
                stats.sum_finder_iters += pbIter;
                return TOIOutput{TOIOutput::e_separated, t2, stats};
            }

            // From here on, t2MinSeparation.distance <= maxTarget

            // Has the separation reached tolerance?
            if (t2MinSeparation.distance >= minTarget)
            {
                if (t2 == t1)
                {
                    //
                    // Can't advance t1 since t2 already the same.
                    //
                    // This state happens when the real distance is greater than maxTarget but the
                    // t2MinSeparation distance is less than maxTarget. If function not stopped,
                    // it runs till stats.toi_iters == conf.maxToiIters and returns
                    // TOIOutput{TOIOutput::e_failed, t1, stats}. Given that the function can't
                    // advance anymore, there's certainly no need to run anymore. Additionally,
                    // given that t1 is the same as t2 and the real distance is separated, this
                    // function can return the separated state.
                    //
                    stats.sum_finder_iters += pbIter;
                    return TOIOutput{TOIOutput::e_separated, t2, stats};
                }

                // Advance the sweeps
                t1 = t2;
                t1xfA = t2xfA;
                t1xfB = t2xfB;
                break;
            }

            // From here on, t2MinSeparation.distance is < minTarget.

            // Compute the initial separation of the witness points.
            const auto t1EvaluatedDistance = fcn.Evaluate(t2MinSeparation.indexPair, t1xfA, t1xfB);

            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            //assert(s1 >= minTarget);
            if (t1EvaluatedDistance < minTarget)
            {
                stats.sum_finder_iters += pbIter;
                return TOIOutput{TOIOutput::e_failed, t1, stats};
            }

            // Check for touching
            if (t1EvaluatedDistance <= maxTarget)
            {
                // Victory! t1 should hold the TOI (could be 0.0).
                stats.sum_finder_iters += pbIter;
                return TOIOutput{TOIOutput::e_touching, t1, stats};
            }

            // Now: t1EvaluatedDistance > maxTarget

            // Compute 1D root of: f(t) - target = 0
            auto a1 = t1;
            auto a2 = t2;
            auto s1 = t1EvaluatedDistance;
            auto s2 = t2MinSeparation.distance;
            auto roots = decltype(conf.maxRootIters){0}; // counts # times f(t) checked
            for (;;)
            {
                assert(!almost_zero((s2 - s1) / Meter));
                assert(a1 <= a2);

                if ((roots == conf.maxRootIters) || (a1 == a2) || (std::nextafter(a1, a2) >= a2))
                {
                    // Reached max root iterations or...
                    // Reached the limit of the Real type's precision!
                    // In this state, there's no way to make progress anymore.
                    // (a1 + a2) / 2 results in a1! So bail from function.
                    stats.sum_finder_iters += pbIter;
                    stats.sum_root_iters += roots;
                    stats.max_root_iters = std::max(stats.max_root_iters, roots);
                    return TOIOutput{TOIOutput::e_failed, a1, stats};
                }

                // Uses secant method to improve convergence (see https://en.wikipedia.org/wiki/Secant_method ).
                // Uses bisection method to guarantee progress (see https://en.wikipedia.org/wiki/Bisection_method ).
                const auto t = (roots & 1)? a1 + (target - s1) * (a2 - a1) / (s2 - s1): (a1 + a2) / 2;
                
                // Using secant method, t may equal a2 now.
                //assert(t != a1);
                ++roots;

                // If t == a1 or t == a2 then, there's a precision/rounding problem.
                // Accept that for now and keep going...

                const auto txfA = GetTransformation(sweepA, t);
                const auto txfB = GetTransformation(sweepB, t);
                const auto s = fcn.Evaluate(t2MinSeparation.indexPair, txfA, txfB);

                if (Abs(s - target) < conf.tolerance) // Root finding succeeded!
                {
                    assert(t != t2);
                    t2 = t; // t2 holds a tentative value for t1
                    t2xfA = txfA;
                    t2xfB = txfB;
                    break; // leave before roots can be == conf.maxRootIters
                }

                // Ensure we continue to bracket the root.
                if (s > target)
                {
                    a1 = t;
                    s1 = s;
                }
                else // s <= target
                {
                    a2 = t;
                    s2 = s;
                }                
            }

            // Found a new t2: t2, t2xfA, and t2xfB have been updated.
            stats.sum_root_iters += roots;
            stats.max_root_iters = std::max(stats.max_root_iters, roots);
        }
        stats.sum_finder_iters += pbIter;
    }

    // stats.toi_iters == conf.maxToiIters
    // Root finder got stuck.
    // This can happen if the two shapes never actually collide within their sweeps.
    return TOIOutput{TOIOutput::e_failed, t1, stats};
}

} // namespace playrho
