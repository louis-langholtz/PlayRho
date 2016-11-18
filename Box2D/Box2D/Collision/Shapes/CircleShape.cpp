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

#include <Box2D/Collision/Shapes/CircleShape.h>
#include <Box2D/Common/BlockAllocator.h>
#include <new>

using namespace box2d;

child_count_t box2d::GetChildCount(const CircleShape& shape)
{
	return 1;
}

bool box2d::TestPoint(const CircleShape& shape, const Transformation& transform, const Vec2& p)
{
	const auto center = transform.p + Rotate(shape.GetLocation(), transform.q);
	return GetLengthSquared(p - center) <= Square(shape.GetRadius());
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
RayCastOutput box2d::RayCast(const CircleShape& shape, const RayCastInput& input,
							 const Transformation& transform, child_count_t childIndex)
{
	BOX2D_NOT_USED(childIndex);

	const auto position = transform.p + Rotate(shape.GetLocation(), transform.q);
	const auto s = input.p1 - position;
	const auto b = GetLengthSquared(s) - Square(shape.GetRadius());

	// Solve quadratic equation.
	const auto r = input.p2 - input.p1;
	const auto c =  Dot(s, r);
	const auto rr = GetLengthSquared(r);
	const auto sigma = Square(c) - rr * b;

	// Check for negative discriminant and short segment.
	if ((sigma < float_t{0}) || almost_zero(rr))
	{
		return RayCastOutput{};
	}

	// Find the point of intersection of the line with the circle.
	const auto a = -(c + Sqrt(sigma));

	// Is the intersection point on the segment?
	if ((a >= float_t{0}) && (a <= (input.maxFraction * rr)))
	{
		const auto fraction = a / rr;
		return RayCastOutput{GetUnitVector(s + fraction * r, UnitVec2::GetZero()), fraction};
	}

	return RayCastOutput{};
}

AABB box2d::ComputeAABB(const CircleShape& shape, const Transformation& transform, child_count_t childIndex)
{
	BOX2D_NOT_USED(childIndex);

	const auto p = transform.p + Rotate(shape.GetLocation(), transform.q);
	return AABB{p, p} + Vec2{shape.GetRadius(), shape.GetRadius()};
}

MassData box2d::ComputeMass(const CircleShape& shape, float_t density)
{
	assert(density >= 0);
	const auto mass = density * Pi * Square(shape.GetRadius());
	const auto I = mass * ((Square(shape.GetRadius()) / float_t{2}) + GetLengthSquared(shape.GetLocation()));
	return MassData{mass, shape.GetLocation(), I};
}
