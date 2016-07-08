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

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/IndexPair.hpp>

namespace box2d
{
	
	/// Used to warm start Distance.
	class SimplexCache
	{
	public:
		/// Maximum count of times this object's add-index method may be called.
		/// @sa AddIndex.
		static constexpr auto MaxCount = unsigned{3};
		
		using size_type = std::remove_const<decltype(MaxCount)>::type;
		
		/// Gets the metric that was set.
		/// @note Behavior is undefined if metric was not previously set.
		///   The IsMetricSet() method can be used to check dynamically if unsure.
		/// @sa SetMetric.
		/// @sa IsMetricSet.
		/// @return Value previously set.
		auto GetMetric() const noexcept
		{
			assert(metric_set);
			return metric;
		}
		
		/// Gets the count.
	 	/// @detail This is the count of times this object's add-index method has been called
		///   since the last clearing of the indexes was done.
		/// @return Value between 0 and MaxCount.
		/// @sa MaxCount.
		/// @sa AddIndex.
		/// @sa ClearIndices.
		auto GetCount() const noexcept { return count; }
		
		auto GetIndexPair(size_type index) const noexcept
		{
			assert(index < count);
			return indexPair[index];
		}
		
		auto GetIndexA(size_type index) const
		{
			assert(index < count);
			return indexPair[index].a;
		}
		
		auto GetIndexB(size_type index) const
		{
			assert(index < count);
			return indexPair[index].b;
		}
		
		void ClearIndices() noexcept { count = 0; }
		
		auto IsMetricSet() const noexcept { return metric_set; }
		
		void SetMetric(float_t m) noexcept
		{
			metric = m;
			metric_set = true;
		}
		
		void AddIndex(IndexPair ip)
		{
			assert(count < MaxCount);
			indexPair[count] = ip;
			++count;
		}
		
	private:
		bool metric_set = false; ///< Whether the metric has been set or not.
		float_t metric; ///< length or area
		size_type count = 0;
		IndexPair indexPair[MaxCount]; ///< Vertices on shape A and B.
	};

}; // namespace box2d

#endif /* SimplexCache_hpp */
