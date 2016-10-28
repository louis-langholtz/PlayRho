/*
 * Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef SimplexVertex_hpp
#define SimplexVertex_hpp

#include <Box2D/Common/Math.h>
#include <Box2D/Collision/IndexPair.hpp>

namespace box2d
{
	/// Simplex vertex.
	///
	/// @note This data structure may be 30-bytes large.
	///
	class SimplexVertex
	{
	public:
		using size_type = IndexPair::size_type;
		
		SimplexVertex() = default;
		
		constexpr SimplexVertex(const SimplexVertex& copy) noexcept = default;
		
		constexpr SimplexVertex(Vec2 sA, size_type iA, Vec2 sB, size_type iB, float_t a_) noexcept;
		
		constexpr SimplexVertex(const SimplexVertex& copy, float_t newA) noexcept;
		
		constexpr Vec2 get_wA() const noexcept { return wA; }
		
		constexpr Vec2 get_wB() const noexcept { return wB; }
		
		constexpr Vec2 get_edge() const noexcept;
		
		/// Gets "A".
		/// @detail This is the "Barycentric coordinate for closest point".
		/// @return Scalar value between 0 and 1 inclusive.
		constexpr float_t get_a() const noexcept { return a; }
		
		/// Sets "A" to the given value.
		/// @note The given value must be between 0 and 1 inclusively. Behavior is undefined otherwise.
		/// @param value Value between 0 and 1 to set "A" to.
		void set_a(float_t value) noexcept
		{
			assert(value >= 0 && value <= 1);
			a = value;
		}
		
		IndexPair indexPair; ///< Indexes of wA and wB. 2-bytes.
		
	private:
		Vec2 wA; ///< Support point in proxy A. 8-bytes.
		Vec2 wB; ///< Support point in proxy B. 8-bytes.
#ifndef DONT_CACHE
		Vec2 e; ///< Edge defined wB - wA. 8-bytes.
#endif
		float_t a; ///< Barycentric coordinate for closest point. 4-bytes.
	};
	
	constexpr inline SimplexVertex::SimplexVertex(Vec2 sA, size_type iA, Vec2 sB, size_type iB, float_t a_) noexcept:
		wA{sA}, wB{sB},
#ifndef DONT_CACHE
		e{wB - wA},
#endif	
		indexPair{iA,iB}, a{a_}
	{
		assert(a_ >= 0 && a_ <= 1);
	}

	constexpr inline SimplexVertex::SimplexVertex(const SimplexVertex& copy, float_t newA) noexcept:
		wA{copy.wA}, wB{copy.wB},
#ifndef DONT_CACHE
		e{copy.e},
#endif	
		indexPair{copy.indexPair}, a{newA}
	{
		assert(newA >= 0 && newA <= 1);
	}

	constexpr inline Vec2 SimplexVertex::get_edge() const noexcept
	{
#ifndef DONT_CACHE
		return e;
#else
		return wB - wA;
#endif			
	}

	/// Gets "w".
	/// @return 2D vector value of wB minus wA.
	constexpr inline Vec2 GetW(const SimplexVertex& sv)
	{
		return sv.get_edge();
	}
	
	constexpr inline Vec2 GetScaledPointA(const SimplexVertex& sv)
	{
		return sv.get_wA() * sv.get_a();
	}
	
	constexpr inline Vec2 GetScaledPointB(const SimplexVertex& sv)
	{
		return sv.get_wB() * sv.get_a();
	}
	
	constexpr inline Vec2 GetScaledDelta(const SimplexVertex& sv)
	{
		return GetW(sv) * sv.get_a();
	}
	
}

#endif /* SimplexVertex_hpp */
