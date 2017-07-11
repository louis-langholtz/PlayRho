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

#ifndef B2_DISTANCE_H
#define B2_DISTANCE_H

#include <PlayRho/Common/Math.hpp>
#include <PlayRho/Collision/Simplex.hpp>

namespace box2d
{
    class DistanceProxy;

    /// Witness Points.
    struct WitnessPoints
    {
        Length2D a;
        Length2D b;
    };
    
    /// Gets the witness points of the given simplex.
    WitnessPoints GetWitnessPoints(const Simplex& simplex) noexcept;
    
    /// Distance Configuration.
    struct DistanceConf
    {
        using iteration_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;

        Simplex::Cache cache;
        iteration_type maxIterations = DefaultMaxDistanceIters;
    };

    /// @brief Distance Output.
    struct DistanceOutput
    {
        enum State: std::uint8_t
        {
            Unknown,
            MaxPoints,
            UnfitSearchDir,
            DuplicateIndexPair,
            HitMaxIters
        };

        using iteration_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;

        Simplex simplex; ///< Simplex.
        iteration_type iterations = 0; ///< Count of iterations performed to return result.
        State state = Unknown; ///< Termination state.
    };

    /// @brief Determines the closest points between two shapes.
    /// @note Supports any combination of shapes.
    /// @note On the first call, the Simplex::Cache.count should be set to zero.
    /// @param proxyA Proxy A.
    /// @param transformA Transoform of A.
    /// @param proxyB Proxy B.
    /// @param transformB Transoform of B.
    /// @param conf Configuration to use including the simplex cache for assisting the determination.
    /// @return Closest points between the two shapes and the count of iterations it took to
    ///   determine them. The iteration count will always be greater than zero unless
    ///   <code>DefaultMaxDistanceIters</code> is zero.
    DistanceOutput Distance(const DistanceProxy& proxyA, const Transformation& transformA,
                            const DistanceProxy& proxyB, const Transformation& transformB,
                            const DistanceConf conf = DistanceConf{});
 
    /// @brief Determine if two generic shapes overlap.
    ///
    /// @note The returned touching state information typically agrees with that returned from
    ///   the CollideShapes function. This is not always the case however especially when the
    ///   separation or overlap distance is closer to zero.
    ///
    Area TestOverlap(const DistanceProxy& proxyA, const Transformation& xfA,
                     const DistanceProxy& proxyB, const Transformation& xfB,
                     const DistanceConf conf = DistanceConf{});

} /* namespace box2d */

#endif
