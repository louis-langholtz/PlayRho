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

#ifndef B2_TIME_OF_IMPACT_H
#define B2_TIME_OF_IMPACT_H

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/Wider.hpp>

namespace box2d {

	class Shape;
	class DistanceProxy;

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
	/// @note A max target of totalRadius - DefaultLinearSlop * x where x is <= 1 is increasingly slower
	///   as x goes below 1.
	/// @note Min target distance needs to be significantly less than the max target distance and
	///   significantly more than 0.
	///
	/// @sa SolvePositionConstraints
 	/// @sa SolveTOIPositionConstraints
	///
	struct ToiConf
	{
		using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
		using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
		using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;

		constexpr ToiConf& UseTimeMax(RealNum value) noexcept;
		constexpr ToiConf& UseTargetDepth(RealNum value) noexcept;
		constexpr ToiConf& UseTolerance(RealNum value) noexcept;
		constexpr ToiConf& UseMaxRootIters(root_iter_type value) noexcept;
		constexpr ToiConf& UseMaxToiIters(toi_iter_type value) noexcept;
		constexpr ToiConf& UseMaxDistIters(dist_iter_type value) noexcept;

		RealNum tMax = 1;
		
		/// Targetted depth of impact.
		/// @note Value must be less than twice the minimum vertex radius of any shape.
		RealNum targetDepth = DefaultLinearSlop * 3;

		RealNum tolerance = DefaultLinearSlop / 4; ///< Tolerance.
		
		/// Maximum number of root finder iterations.
		/// @detail This is the maximum number of iterations for calculating the 1D root of
		///    <code>f(t) - (totalRadius - targetDepth) < tolerance</code>
		/// where <code>f(t)</code> is the distance between the shapes at time <code>t</code>,
		/// and <code>totalRadius</code> is the sum of the vertex radiuses of 2 distance proxies.
		/// @note This value never needs to be more than the number of iterations needed to
		///    achieve full machine precision.
		root_iter_type maxRootIters = DefaultMaxToiRootIters;
		
		toi_iter_type maxToiIters = DefaultMaxToiIters; ///< Max time of impact iterations.
		
		dist_iter_type maxDistIters = DefaultMaxDistanceIters;
	};

	constexpr auto GetDefaultToiConf()
	{
		return ToiConf{};
	}

	constexpr ToiConf& ToiConf::UseTimeMax(RealNum value) noexcept
	{
		tMax = value;
		return *this;
	}

	constexpr ToiConf& ToiConf::UseTargetDepth(RealNum value) noexcept
	{
		targetDepth = value;
		return *this;
	}

	constexpr ToiConf& ToiConf::UseTolerance(RealNum value) noexcept
	{
		tolerance = value;
		return *this;
	}

	constexpr ToiConf& ToiConf::UseMaxRootIters(root_iter_type value) noexcept
	{
		maxRootIters = value;
		return *this;
	}
	
	constexpr ToiConf& ToiConf::UseMaxToiIters(toi_iter_type value) noexcept
	{
		maxToiIters = value;
		return *this;
	}

	constexpr ToiConf& ToiConf::UseMaxDistIters(dist_iter_type value) noexcept
	{
		maxDistIters = value;
		return *this;
	}

	/// TimeOfImpact Output data.
	class TOIOutput
	{
	public:
		using toi_iter_type = std::remove_const<decltype(DefaultMaxToiIters)>::type;
		using dist_iter_type = std::remove_const<decltype(DefaultMaxDistanceIters)>::type;
		using root_iter_type = std::remove_const<decltype(DefaultMaxToiRootIters)>::type;
		using toi_sum_type = Wider<toi_iter_type>::type;
		using dist_sum_type = Wider<dist_iter_type>::type;
		using root_sum_type = Wider<root_iter_type>::type;

		struct Stats
		{
			// 3-bytes
			toi_iter_type toi_iters = 0; ///< Time of impact iterations.
			dist_iter_type max_dist_iters = 0; ///< Max. distance iterations count.
			root_iter_type max_root_iters = 0; ///< Max. root finder iterations for all TOI iterations.

			// 4-bytes
			toi_sum_type sum_finder_iters = 0; ///< Sum total TOI iterations.
			dist_sum_type sum_dist_iters = 0; ///< Sum total distance iterations.
			root_sum_type sum_root_iters = 0; ///< Sum total of root finder iterations.
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
		
		constexpr TOIOutput(State state, RealNum time, Stats stats): m_state(state), m_time(time), m_stats(stats)
		{
			assert(time >= 0);
			assert(time <= 1);
		}

		/// Gets the state at time factor.
		State get_state() const noexcept { return m_state; }

		/// Gets time factor at which state occurs.
		/// @return Time factor in range of [0,1] into the future.
		RealNum get_t() const noexcept { return m_time; }

		toi_iter_type get_toi_iters() const noexcept { return m_stats.toi_iters; }
		
		dist_sum_type get_sum_dist_iters() const noexcept { return m_stats.sum_dist_iters; }
		
		dist_iter_type get_max_dist_iters() const noexcept { return m_stats.max_dist_iters; }

		root_sum_type get_sum_root_iters() const noexcept { return m_stats.sum_root_iters; }
		
		root_iter_type get_max_root_iters() const noexcept { return m_stats.max_root_iters; }
		
	private:
		State m_state = e_unknown; ///< State at time factor.
		RealNum m_time = 0; ///< Time factor in range of [0,1] into the future.
		Stats m_stats;
	};

	/// Calculates the time of impact.
	/// @detail
	/// Computes the upper bound on time before two shapes penetrate too much.
	/// Time is represented as a fraction between [0,tMax].
	/// This uses a swept separating axis and may miss some intermediate,
	/// non-tunneling collision.
	/// If you change the time interval, you should call this function again.
	/// @pre The given sweeps are both at the same alpha0.
	/// @warning Behavior is undefined if the given sweeps are not at the same alpha0.
	/// @note Uses Distance to compute the contact point and normal at the time of impact.
	/// @param proxyA Proxy A. The proxy's vertex count must be 1 or more.
	/// @param sweepA Sweep A. Sweep of motion for shape represented by proxy A.
	/// @param proxyB Proxy B. The proxy's vertex count must be 1 or more.
	/// @param sweepB Sweep B. Sweep of motion for shape represented by proxy B.
	/// @param conf Configuration details for on calculation. Like the targetted depth of penetration.
	/// @return Time of impact output data.
	TOIOutput TimeOfImpact(const DistanceProxy& proxyA, const Sweep& sweepA,
						   const DistanceProxy& proxyB, const Sweep& sweepB,
						   const ToiConf conf = GetDefaultToiConf());

} // namespace box2d

#endif
