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
	/// @detail This is the locations (in world coordinates) and indices of a pair of vertices
	/// from two shapes (shape A and shape B).
	///
	/// @note This data structure may be 28-bytes large.
	///
	class SimplexVertex
	{
	public:
		using size_type = IndexPair::size_type;
		
		/// Default constructor.
		SimplexVertex() = default;
		
		constexpr SimplexVertex(const SimplexVertex& copy) noexcept = default;
		
		/// Initializing constructor.
		/// @param pA Point A in world coordinates.
		/// @param iA Index of point A within the shape that it comes from.
		/// @param pB Point B in world coordinates.
		/// @param iB Index of point B within the shape that it comes from.
		constexpr SimplexVertex(Vec2 pA, size_type iA, Vec2 pB, size_type iB) noexcept;
		
		/// Gets point A (in world coordinates).
		constexpr Vec2 GetPointA() const noexcept { return m_wA; }
		
		/// Gets point B (in world coordinates).
		constexpr Vec2 GetPointB() const noexcept { return m_wB; }
		
		/// Gets the point delta.
		/// @detail This is the difference between points A and B.
		/// @return Point B minus point A.
		constexpr Vec2 GetPointDelta() const noexcept;
				
		IndexPair indexPair; ///< Index pair. @detail Indices of points A and B. 2-bytes.
		
	private:
		Vec2 m_wA; ///< Point A in world coordinates. This is the support point in proxy A. 8-bytes.
		Vec2 m_wB; ///< Point B in world coordinates. This is the support point in proxy B. 8-bytes.
#ifndef DONT_CACHE
		Vec2 m_delta; ///< Edge defined wB - wA. 8-bytes.
#endif
	};
	
	constexpr inline SimplexVertex::SimplexVertex(Vec2 pA, size_type iA, Vec2 pB, size_type iB) noexcept:
		m_wA{pA}, m_wB{pB},
#ifndef DONT_CACHE
		m_delta{pB - pA},
#endif	
		indexPair{iA,iB}
	{
	}

	constexpr inline Vec2 SimplexVertex::GetPointDelta() const noexcept
	{
#ifndef DONT_CACHE
		return m_delta;
#else
		return m_wB - m_wA;
#endif			
	}

	/// Gets "w".
	/// @return 2D vector value of wB minus wA.
	constexpr inline Vec2 GetPointDelta(const SimplexVertex& sv)
	{
		return sv.GetPointDelta();
	}
	
	constexpr inline bool operator == (const SimplexVertex& lhs, const SimplexVertex& rhs)
	{
		return (lhs.GetPointA() == rhs.GetPointA())
			&& (lhs.GetPointB() == rhs.GetPointB())
			&& (lhs.indexPair == rhs.indexPair);
	}
	
	constexpr inline bool operator != (const SimplexVertex& lhs, const SimplexVertex& rhs)
	{
		return !(lhs == rhs);
	}
}

#endif /* SimplexVertex_hpp */
