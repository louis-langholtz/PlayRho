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
#include <new>

using namespace box2d;

Shape* CircleShape::Clone(BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(CircleShape));
	return new (mem) CircleShape(*this);
}

child_count_t CircleShape::GetChildCount() const
{
	return 1;
}

bool CircleShape::TestPoint(const Transformation& transform, const Vec2& p) const
{
	const auto center = transform.p + Rotate(GetPosition(), transform.q);
	return LengthSquared(p - center) <= Square(GetRadius());
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool CircleShape::RayCast(RayCastOutput* output, const RayCastInput& input,
							const Transformation& transform, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	const auto position = transform.p + Rotate(m_p, transform.q);
	const auto s = input.p1 - position;
	const auto b = LengthSquared(s) - Square(GetRadius());

	// Solve quadratic equation.
	const auto r = input.p2 - input.p1;
	const auto c =  Dot(s, r);
	const auto rr = LengthSquared(r);
	const auto sigma = Square(c) - rr * b;

	// Check for negative discriminant and short segment.
	if ((sigma < float_t{0}) || (rr < Epsilon))
	{
		return false;
	}

	// Find the point of intersection of the line with the circle.
	const auto a = -(c + Sqrt(sigma));

	// Is the intersection point on the segment?
	if ((float_t{0} <= a) && (a <= (input.maxFraction * rr)))
	{
		const auto fraction = a / rr;
		output->fraction = fraction;
		output->normal = GetUnitVector(s + fraction * r);
		return true;
	}

	return false;
}

AABB CircleShape::ComputeAABB(const Transformation& transform, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	const auto p = transform.p + Rotate(m_p, transform.q);
	return AABB{p, p} + Vec2{GetRadius(), GetRadius()};
}

MassData CircleShape::ComputeMass(float_t density) const
{
	assert(density >= 0);
	const auto mass = density * Pi * Square(GetRadius());
	const auto I = mass * ((Square(GetRadius()) / float_t{2}) + LengthSquared(m_p));
	return MassData{mass, m_p, I};
}
