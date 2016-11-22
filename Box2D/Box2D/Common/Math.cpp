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

#include <Box2D/Common/Math.hpp>

using namespace box2d;

Vec2 box2d::ComputeCentroid(const Span<const Vec2>& vertices)
{
	assert(vertices.size() >= 3);
	
	auto c = Vec2_zero;
	auto area = float_t{0};
	
	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	const auto pRef = Average(vertices);

	for (auto i = decltype(vertices.size()){0}; i < vertices.size(); ++i)
	{
		// Triangle vertices.
		const auto p1 = pRef;
		const auto p2 = vertices[i];
		const auto p3 = vertices[(i + 1) % vertices.size()];
		
		const auto e1 = p2 - p1;
		const auto e2 = p3 - p1;
				
		const auto triangleArea = Cross(e1, e2) / 2;
		area += triangleArea;
		
		// Area weighted centroid
		c += triangleArea * (p1 + p2 + p3) / 3;
	}
	
	// Centroid
	assert((area > 0) && !almost_zero(area));
	return c / area;
}

UnitVec2::UnitVec2(Vec2 value, UnitVec2 fallback) noexcept
{
	if (IsValid(value))
	{
		const auto lengthSquared = GetLengthSquared(value);
		if (lengthSquared >= std::numeric_limits<decltype(lengthSquared)>::min())
		{
			const auto unitized = value / Sqrt(lengthSquared);
			m_x = unitized.x;
			m_y = unitized.y;
			return;
		}
		m_x = fallback.GetX();
		m_y = fallback.GetY();
		return;
	}
	m_x = GetInvalid<data_type>();
	m_y = GetInvalid<data_type>();
}

::std::ostream& box2d::operator<<(::std::ostream& os, const Vec2& value)
{
	return os << "Vec2(" << value.x << "," << value.y << ")";
}
