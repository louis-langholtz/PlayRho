/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <new>

using namespace box2d;

Shape* b2CircleShape::Clone(BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2CircleShape));
	return new (mem) b2CircleShape(*this);
}

child_count_t b2CircleShape::GetChildCount() const
{
	return 1;
}

bool b2CircleShape::TestPoint(const Transform& transform, const Vec2& p) const
{
	const auto center = transform.p + Mul(transform.q, m_p);
	const auto d = p - center;
	return d.LengthSquared() <= Square(GetRadius());
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool b2CircleShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const Transform& transform, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	const auto position = transform.p + Mul(transform.q, m_p);
	const auto s = input.p1 - position;
	const auto b = s.LengthSquared() - Square(GetRadius());

	// Solve quadratic equation.
	const auto r = input.p2 - input.p1;
	const auto c =  Dot(s, r);
	const auto rr = r.LengthSquared();
	const auto sigma = Square(c) - rr * b;

	// Check for negative discriminant and short segment.
	if ((sigma < float_t{0}) || (rr < Epsilon))
	{
		return false;
	}

	// Find the point of intersection of the line with the circle.
	auto a = -(c + Sqrt(sigma));

	// Is the intersection point on the segment?
	if ((float_t{0} <= a) && (a <= (input.maxFraction * rr)))
	{
		a /= rr;
		output->fraction = a;
		output->normal = Normalize(s + a * r);
		return true;
	}

	return false;
}

AABB b2CircleShape::ComputeAABB(const Transform& transform, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	const auto p = transform.p + Mul(transform.q, m_p);
	return AABB{p, p} + Vec2{GetRadius(), GetRadius()};
}

MassData b2CircleShape::ComputeMass(float_t density) const
{
	const auto mass = density * Pi * Square(GetRadius());
	const auto I = mass * ((Square(GetRadius()) / float_t(2)) + m_p.LengthSquared());
	return MassData{mass, m_p, I};
}
