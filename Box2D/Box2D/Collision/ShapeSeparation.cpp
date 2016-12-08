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

ShapeSeparation box2d::GetMaxSeparation(Span<const Vec2> vertices1, Span<const UnitVec2> normals1, const Transformation& xf1,
										Span<const Vec2> vertices2, const Transformation& xf2)
{
	assert(vertices1.size() == normals1.size());
	
	// Find the max separation between shape1 and shape2 using edge normals from shape1.
	auto maxSeparation = -MaxFloat;
	auto index_of_max = ShapeSeparation::index_type{0};
	
	const auto count1 = vertices1.size();
	const auto xf = MulT(xf2, xf1);
	
	for (auto i = decltype(count1){0}; i < count1; ++i)
	{
		// Get shape1 normal and vertex relative to shape2.
		const auto shape1_ni = Rotate(normals1[i], xf.q);
		const auto shape1_vi = Transform(vertices1[i], xf);
		
		const auto s = GetMostOppositeSeparation(vertices2, Vec2{shape1_ni}, shape1_vi).separation;
		if (maxSeparation < s)
		{
			maxSeparation = s;
			index_of_max = static_cast<ShapeSeparation::index_type>(i);
		}
	}
	return ShapeSeparation{index_of_max, maxSeparation};
}
