/*
 * Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef StepStats_hpp
#define StepStats_hpp

#include <Box2D/Common/Settings.hpp>

namespace box2d {
	
	/// Pre-phase per-step statistics.
	struct PreStepStats
	{
		uint32 ignored = 0;
		uint32 destroyed = 0;
		uint32 updated = 0;
		uint32 added = 0;
	};
	
	/// Regular-phase per-step statistics.
	struct RegStepStats
	{
		Length minSeparation = std::numeric_limits<RealNum>::infinity() * Meter;
		Momentum maxIncImpulse = 0;
		
		uint32 islandsFound = 0;
		uint32 islandsSolved = 0;
		uint32 contactsAdded = 0;
		uint32 bodiesSlept = 0;
		uint32 proxiesMoved = 0;
		uint32 sumPosIters = 0;
		uint32 sumVelIters = 0;
	};
	
	/// TOI-phase per-step statistics.
	struct ToiStepStats
	{
		Length minSeparation = std::numeric_limits<RealNum>::infinity() * Meter;
		Momentum maxIncImpulse = 0;
		
		uint32 islandsFound = 0;
		uint32 islandsSolved = 0;
		uint32 contactsFound = 0;
		uint32 contactsAtMaxSubSteps = 0;
		uint32 contactsUpdatedToi = 0;
		uint32 contactsAdded = 0;
		uint32 proxiesMoved = 0;
		uint32 sumPosIters = 0;
		uint32 sumVelIters = 0;
		uint32 maxSimulContacts = 0; ///< Max contacts occuring simultaneously.
		
		using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;
		using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
		using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
		
		dist_iter_type maxDistIters = 0;
		toi_iter_type maxToiIters = 0;
		root_iter_type maxRootIters = 0;
	};
	
	/// Per-step statistics.
	///
	/// @details These are statistics output from the World::Step method.
	/// @note Efficient transfer of this data is predicated on compiler support for
	///   "return-value-optimization" - a form of "copy-elision".
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
	
} // namespace box2d

#endif /* StepStats_hpp */
