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

#ifndef ReferenceFace_hpp
#define ReferenceFace_hpp

#include <Box2D/Common/Math.hpp>

namespace box2d
{
	class EdgeInfo;
	class PolygonShape;

	// Reference face used for clipping
	class ReferenceFace
	{
	public:
		using index_type = std::remove_const<decltype(MaxShapeVertices)>::type;
		
		ReferenceFace(index_type i1, Vec2 v1, index_type i2, Vec2 v2, UnitVec2 normal) noexcept;
		
		index_type GetIndex1() const noexcept { return m_idx1; };
		Vec2 GetVertex1() const noexcept { return m_v1; };
		UnitVec2 GetNormal1() const noexcept { return m_normal1; };
		realnum GetOffset1() const noexcept { return m_offset1; };
		
		index_type GetIndex2() const noexcept { return m_idx2; };
		Vec2 GetVertex2() const noexcept { return m_v2; };
		UnitVec2 GetNormal2() const noexcept { return m_normal2; };
		realnum GetOffset2() const noexcept { return m_offset2; };
		
		UnitVec2 GetNormal() const noexcept { return m_normal; };
		
	private:
		// Keeps similar sized fields together for potentially better space utilization
		
		UnitVec2 m_normal;
		
		Vec2 m_v1;
		Vec2 m_v2;
		
		UnitVec2 m_normal1;
		UnitVec2 m_normal2;
		
		realnum m_offset1;	
		realnum m_offset2;
		
		index_type m_idx1;
		index_type m_idx2;
	};
	
	inline ReferenceFace::ReferenceFace(index_type i1, Vec2 v1, index_type i2, Vec2 v2, UnitVec2 normal) noexcept:
		m_idx1{i1},
		m_idx2{i2},
		m_v1{v1},
		m_v2{v2},
		m_normal{normal},
		m_normal1{GetFwdPerpendicular(normal)},
		m_normal2{-m_normal1},
		m_offset1{Dot(m_normal1, m_v1)},
		m_offset2{Dot(m_normal2, m_v2)}
	{
		// Intentionally empty.
	}

	ReferenceFace GetReferenceFace(const EdgeInfo& edgeInfo);
	ReferenceFace GetReferenceFace(const PolygonShape& localShapeB, const ReferenceFace::index_type index);
}

#endif /* ReferenceFace_hpp */
