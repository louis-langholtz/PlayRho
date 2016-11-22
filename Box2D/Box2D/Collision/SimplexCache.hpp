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

#ifndef SimplexCache_hpp
#define SimplexCache_hpp

#include <Box2D/Common/Math.hpp>
#include <Box2D/Common/ArrayList.hpp>
#include <Box2D/Collision/IndexPairList.hpp>

namespace box2d
{
	/// Simplex cache.
	///
	/// @detail Used to warm start Distance.
	/// Caches particular information from a simplex - a related metric and up-to 3 index pairs.
	///
	/// @invariant As the metric and list of index pairs should be values from a snapshot of a
	///   simplex, the mertic and list of index pairs must not vary independent of each other.
	///   As such, this data structure only allows these values to be changed in unision via object
	///   construction or object assignment.
	///
	/// @note This data structure is 12-bytes large.
	///
	class SimplexCache
	{
	public:

		using size_type = IndexPairList::size_type;

		SimplexCache() = default;

		SimplexCache(const SimplexCache& copy) = default;
		
		constexpr SimplexCache(float_t metric, IndexPairList indices) noexcept:
			m_metric{metric}, m_indices{indices}
		{
			// Intentionally empty
		}

		/// Gets the metric that was set.
		/// @note Behavior is undefined if metric was not previously set.
		///   The IsMetricSet() method can be used to check dynamically if unsure.
		/// @sa SetMetric.
		/// @sa IsMetricSet.
		/// @return Value previously set.
		auto GetMetric() const noexcept
		{
			assert(IsValid(m_metric));
			return m_metric;
		}
				
		auto IsMetricSet() const noexcept { return IsValid(m_metric); }
		
		auto GetIndices() const noexcept { return m_indices; }

		auto GetNumIndices() const noexcept { return m_indices.size(); }

		auto GetIndexPair(size_type index) const noexcept { return m_indices[index]; }

	private:
		float_t m_metric = GetInvalid<float_t>(); ///< length or area

		/// Indices.
		/// @detail
		/// Collection of index-pairs.
		IndexPairList m_indices;
	};

}; // namespace box2d

#endif /* SimplexCache_hpp */
