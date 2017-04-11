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

#ifndef EdgeInfo_hpp
#define EdgeInfo_hpp

#include <Box2D/Common/Math.hpp>

namespace box2d
{
	class EdgeShape;

	class EdgeInfo
	{
	public:
		EdgeInfo() = default;
		
		EdgeInfo(const EdgeShape& edge, const Length2D centroid) noexcept;
		
		Length2D GetVertex1() const noexcept { return m_vertex1; }
		Length2D GetVertex2() const noexcept { return m_vertex2; }
		UnitVec2 GetEdge1() const noexcept { return m_edge1; }
		UnitVec2 GetNormal1() const noexcept { return m_normal1; }
		
		bool IsFront() const noexcept { return m_front; }
		
		/// Gets the normal.
		/// @return Value of normal 1 or the negative of it depending on whether
		///   is-front is true or not (respectively).
		UnitVec2 GetNormal() const noexcept
		{
			// Alternatively:
			//   return m_front? m_normal1: -m_normal1;
			return m_normal;
		}
		
		UnitVec2 GetLowerLimit() const noexcept { return m_lowerLimit; }
		
		UnitVec2 GetUpperLimit() const noexcept { return m_upperLimit; }
		
		Length GetVertexRadius() const noexcept { return m_vertexRadius; }
		
	private:
		Length2D m_vertex1;
		Length2D m_vertex2;
		UnitVec2 m_edge1; ///< Edge 1. @detail A unit vector of edge shape's vertex 2 - vertex 1.
		UnitVec2 m_normal1; ///< Normal 1. @detail Forward perpendicular of edge 1.
		
		bool m_front;
		
		/// Normal.
		/// @detail This is the cached value of <code>m_normal1</code> or the negative of it depending
		///   on whether <code>m_front</code> is true or not (respectively).
		UnitVec2 m_normal;
		
		UnitVec2 m_lowerLimit;
		UnitVec2 m_upperLimit;
		
		Length m_vertexRadius;
		
		void SetNormalLowerUpper(UnitVec2 normal, UnitVec2 lower, UnitVec2 upper) noexcept
		{
			m_normal = normal;
			m_lowerLimit = lower;
			m_upperLimit = upper;
		}
	};

} // namespace box2d

#endif /* EdgeInfo_hpp */
