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

#include <Box2D/Collision/ShapeSeparation.hpp>

using namespace box2d;

IndexPairSeparation box2d::GetMaxSeparation(Span<const Vec2> verts1, Span<const UnitVec2> norms1, const Transformation& xf1,
										Span<const Vec2> verts2, const Transformation& xf2,
										RealNum stop)
{
	assert(verts1.size() == norms1.size());
	
	// Find the max separation between shape1 and shape2 using edge normals from shape1.

	auto indexPairSep = IndexPairSeparation{-MaxFloat, IndexPairSeparation::InvalidIndex, IndexPairSeparation::InvalidIndex};
	const auto xf = MulT(xf2, xf1);
	const auto count1 = verts1.size();
	for (auto i = decltype(count1){0}; i < count1; ++i)
	{
		// Get shape1 normal and vertex relative to shape2.
		const auto s = GetMostAntiParallelSeparation(verts2, Vec2{Rotate(norms1[i], xf.q)}, Transform(verts1[i], xf));
		if (s.separation > stop)
		{
			return IndexPairSeparation{s.separation, static_cast<IndexSeparation::index_type>(i), s.index};
		}
		if (indexPairSep.separation < s.separation)
		{
			indexPairSep = IndexPairSeparation{s.separation, static_cast<IndexSeparation::index_type>(i), s.index};
		}
	}
	return indexPairSep;
}

IndexPairSeparation box2d::GetMaxSeparation(Span<const Vec2> verts1, Span<const UnitVec2> norms1,
										Span<const Vec2> verts2,
										RealNum stop)
{
	assert(verts1.size() == norms1.size());
	
	// Find the max separation between shape1 and shape2 using edge normals from shape1.
	
	auto indexPairSep = IndexPairSeparation{-MaxFloat, IndexPairSeparation::InvalidIndex, IndexPairSeparation::InvalidIndex};
	const auto count1 = verts1.size();
	for (auto i = decltype(count1){0}; i < count1; ++i)
	{
		const auto s = GetMostAntiParallelSeparation(verts2, Vec2{norms1[i]}, verts1[i]);
		if (s.separation > stop)
		{
			return IndexPairSeparation{s.separation, static_cast<IndexSeparation::index_type>(i), s.index};
		}
		if (indexPairSep.separation < s.separation)
		{
			indexPairSep = IndexPairSeparation{s.separation, static_cast<IndexSeparation::index_type>(i), s.index};
		}
	}
	return indexPairSep;
}
