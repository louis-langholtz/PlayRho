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

#include <Box2D/Collision/Shapes/CircleShape.hpp>
#include <Box2D/Collision/RayCastInput.hpp>

using namespace box2d;

MassData CircleShape::GetMassData() const noexcept
{
	return ::GetMassData(GetVertexRadius(), GetDensity(), GetLocation());
}

bool CircleShape::TestPoint(const Transformation& transform, const Length2D p) const noexcept
{
	const auto location = GetLocation();
	const auto center = transform.p + Rotate(location, transform.q);
	const auto delta = p - center;
	return GetLengthSquared(delta) <= Square(GetRadius());
}

RayCastOutput CircleShape::RayCast(const RayCastInput& input, const Transformation& transform,
								  child_count_t childIndex) const noexcept
{
	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.1.2
	// x = s + a * r
	// norm(x) = radius

	NOT_USED(childIndex);
	
	const auto loc = GetLocation();
	const auto position = transform.p + Rotate(loc, transform.q);
	const auto s = input.p1 - position;
	const auto sUnitless = StripUnits(s);
	const auto b = GetLengthSquared(sUnitless) - Square(GetRadius() / Meter);
	
	// Solve quadratic equation.
	const auto r = input.p2 - input.p1;
	const auto rUnitless = StripUnits(r);
	const auto c =  Dot(sUnitless, rUnitless);
	const auto rr = GetLengthSquared(rUnitless);
	const auto sigma = Square(c) - rr * b;
	
	// Check for negative discriminant and short segment.
	if ((sigma < RealNum{0}) || almost_zero(rr))
	{
		return RayCastOutput{};
	}
	
	// Find the point of intersection of the line with the circle.
	const auto a = -(c + Sqrt(sigma));
	
	// Is the intersection point on the segment?
	if ((a >= RealNum{0}) && (a <= (input.maxFraction * rr)))
	{
		const auto fraction = a / rr;
		return RayCastOutput{GetUnitVector(sUnitless + fraction * rUnitless, UnitVec2::GetZero()), fraction};
	}
	
	return RayCastOutput{};
}
