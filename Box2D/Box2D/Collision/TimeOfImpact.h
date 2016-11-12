/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef B2_TIME_OF_IMPACT_H
#define B2_TIME_OF_IMPACT_H

#include <Box2D/Common/Math.h>

namespace box2d {

	class Shape;
	class DistanceProxy;

	/// Determine if two generic shapes overlap.
	bool TestOverlap(const Shape& shapeA, child_count_t indexA, const Transformation& xfA,
					 const Shape& shapeB, child_count_t indexB, const Transformation& xfB);

	/// Time of impact configuration.
	///
	/// @detail These parameters effect time of impact calculations by limiting the definitions
	///    of time and impact. If total radius is expressed as TR, and target depth as TD, then:
	///    the max target distance is (TR - TD) + tolerance; and the min target distance is
	///    (TR - TD) - tolerance.
	///
	/// @note Max target distance must be less than or equal to the total radius as the target
	///   range has to be chosen such that the contact manifold will have a greater than zero
	///   contact point count.
	/// @note A max target of totalRadius - LinearSlop * x where x is <= 1 is increasingly slower
	///   as x goes below 1.
	/// @note Min target distance needs to be significantly less than the max target distance and
	///   significantly more than 0.
	///
	/// @sa SolvePositionConstraints
 	/// @sa SolveTOIPositionConstraints
	///
	struct TOILimits
	{
		using root_iter_type = std::remove_const<decltype(MaxTOIRootIterCount)>::type;
		using toi_iter_type = std::remove_const<decltype(MaxTOIIterations)>::type;

		float_t tMax = 1;
		float_t targetDepth = LinearSlop * 3; ///< Targetted depth of impact.
		float_t tolerance = LinearSlop / 4; ///< Tolerance.
		
		root_iter_type maxRootIters = MaxTOIRootIterCount;
		toi_iter_type maxToiIters = MaxTOIIterations; ///< Max time of impact iterations.
	};

	constexpr auto GetDefaultTOILimits()
	{
		return TOILimits{};
	}

	/// TimeOfImpact Output data.
	class TOIOutput
	{
	public:
		using toi_iter_type = std::remove_const<decltype(MaxTOIIterations)>::type;
		using dist_iter_type = std::remove_const<decltype(MaxDistanceIterations)>::type;
		using root_iter_type = std::remove_const<decltype(MaxTOIRootIterCount)>::type;
		using dist_sum_type = std::conditional<sizeof(dist_iter_type) < sizeof(uint16), uint16, uint32>::type;
		using root_sum_type = std::conditional<sizeof(root_iter_type) < sizeof(uint16), uint16, uint32>::type;

		struct Stats
		{
			Stats() = default;

			constexpr Stats(toi_iter_type toi,
							dist_sum_type dist_sum, dist_iter_type dist_max,
							root_sum_type root_sum, root_iter_type root_max) noexcept:
				toi_iters{toi},
				max_dist_iters{dist_max},
				max_root_iters{root_max},
				sum_dist_iters{dist_sum},
				sum_root_iters{root_sum}
			{}

			// 3-bytes
			toi_iter_type toi_iters; ///< Time of impact iterations.
			dist_iter_type max_dist_iters; ///< Max. distance iterations count.
			root_iter_type max_root_iters; ///< Max. root finder iterations for all TOI iterations.

			// 4-bytes
			dist_sum_type sum_dist_iters; ///< Sum total distance iterations.
			root_sum_type sum_root_iters; ///< Sum total of root finder iterations.
		};

		enum State: uint16
		{
			e_unknown,
			e_failed,
			e_overlapped,
			e_touching,
			e_separated
		};

		TOIOutput() = default;
		
		constexpr TOIOutput(State state, float_t time, Stats stats): m_state(state), m_time(time), m_stats(stats)
		{
			assert(time >= 0);
			assert(time <= 1);
		}

		/// Gets the state at time factor.
		State get_state() const noexcept { return m_state; }

		/// Gets time factor at which state occurs.
		/// @return Time factor in range of [0,1] into the future.
		float_t get_t() const noexcept { return m_time; }

		toi_iter_type get_toi_iters() const noexcept { return m_stats.toi_iters; }
		
		dist_sum_type get_sum_dist_iters() const noexcept { return m_stats.sum_dist_iters; }
		
		dist_iter_type get_max_dist_iters() const noexcept { return m_stats.max_dist_iters; }

		root_sum_type get_sum_root_iters() const noexcept { return m_stats.sum_root_iters; }
		
		root_iter_type get_max_root_iters() const noexcept { return m_stats.max_root_iters; }
		
	private:
		State m_state = e_unknown; ///< State at time factor.
		float_t m_time; ///< Time factor in range of [0,1] into the future.
		Stats m_stats;
	};

	/// Calculates the time of impact.
	/// @detail
	/// Computes the upper bound on time before two shapes penetrate too much.
	/// Time is represented as a fraction between [0,tMax].
	/// This uses a swept separating axis and may miss some intermediate,
	/// non-tunneling collision.
	/// If you change the time interval, you should call this function again.
	/// @note Uses Distance to compute the contact point and normal at the time of impact.
	/// @param proxyA Proxy A. The proxy's vertex count must be 1 or more.
	/// @param sweepA Sweep A. Sweep of motion for shape represented by proxy A.
	/// @param proxyB Proxy B. The proxy's vertex count must be 1 or more.
	/// @param sweepB Sweep B. Sweep of motion for shape represented by proxy B.
	/// @param limits Limits on calculation. Like the targetted depth of penetration.
	/// @return Time of impact output data.
	TOIOutput TimeOfImpact(const DistanceProxy& proxyA, const Sweep& sweepA,
						   const DistanceProxy& proxyB, const Sweep& sweepB,
						   const TOILimits limits = GetDefaultTOILimits());

} // namespace box2d

#endif
