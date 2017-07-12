/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef StepStats_hpp
#define StepStats_hpp

#include <PlayRho/Common/Settings.hpp>

namespace playrho {
    
    /// @brief Pre-phase per-step statistics.
    struct PreStepStats
    {
        using counter_type = std::uint32_t;
        counter_type proxiesMoved = 0;
        counter_type ignored = 0;
        counter_type destroyed = 0;
        counter_type updated = 0;
        counter_type skipped = 0;
        counter_type added = 0;
    };
    
    /// @brief Regular-phase per-step statistics.
    struct RegStepStats
    {
        using counter_type = std::uint32_t;

        Length minSeparation = std::numeric_limits<Real>::infinity() * Meter;
        Momentum maxIncImpulse = 0;
        
        counter_type islandsFound = 0;
        counter_type islandsSolved = 0;
        counter_type contactsAdded = 0;
        counter_type bodiesSlept = 0;
        counter_type proxiesMoved = 0;
        counter_type sumPosIters = 0;
        counter_type sumVelIters = 0;
    };
    
    /// @brief TOI-phase per-step statistics.
    struct ToiStepStats
    {
        using counter_type = std::uint32_t;

        Length minSeparation = std::numeric_limits<Real>::infinity() * Meter;
        Momentum maxIncImpulse = 0;
        
        counter_type islandsFound = 0;
        counter_type islandsSolved = 0;
        counter_type contactsFound = 0;
        counter_type contactsAtMaxSubSteps = 0;
        counter_type contactsUpdatedToi = 0;
        counter_type contactsUpdatedTouching = 0;
        counter_type contactsSkippedTouching = 0;
        counter_type contactsAdded = 0;
        counter_type proxiesMoved = 0;
        counter_type sumPosIters = 0;
        counter_type sumVelIters = 0;
        counter_type maxSimulContacts = 0; ///< Max contacts occuring simultaneously.
        
        using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;
        using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
        using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
        
        dist_iter_type maxDistIters = 0;
        toi_iter_type maxToiIters = 0;
        root_iter_type maxRootIters = 0;
    };
    
    /// @brief Per-step statistics.
    ///
    /// @details These are statistics output from the World::Step method.
    /// @note Efficient transfer of this data is predicated on compiler support for
    ///   "return-value-optimization" - a form of "copy elision".
    ///
    /// @sa World::Step.
    /// @sa https://en.wikipedia.org/wiki/Return_value_optimization
    /// @sa http://en.cppreference.com/w/cpp/language/copy_elision
    ///
    struct StepStats
    {
        PreStepStats pre; ///< Pre-phase step statistics.
        RegStepStats reg; ///< Reg-phase step statistics.
        ToiStepStats toi; ///< TOI-phase step statistics.
    };
    
} // namespace playrho

#endif /* StepStats_hpp */
