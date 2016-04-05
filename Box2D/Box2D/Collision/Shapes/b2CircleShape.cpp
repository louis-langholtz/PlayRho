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

b2Shape* b2CircleShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2CircleShape));
	return new (mem) b2CircleShape(*this);
}

int32 b2CircleShape::GetChildCount() const
{
	return 1;
}

bool b2CircleShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
{
	const auto center = transform.p + b2Mul(transform.q, m_p);
	const auto d = p - center;
	return b2Dot(d, d) <= (GetRadius() * GetRadius());
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool b2CircleShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	const auto position = transform.p + b2Mul(transform.q, m_p);
	const auto s = input.p1 - position;
	const auto b = b2Dot(s, s) - (GetRadius() * GetRadius());

	// Solve quadratic equation.
	const auto r = input.p2 - input.p1;
	const auto c =  b2Dot(s, r);
	const auto rr = b2Dot(r, r);
	const auto sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if ((sigma < 0.0f) || (rr < b2_epsilon))
	{
		return false;
	}

	// Find the point of intersection of the line with the circle.
	auto a = -(c + b2Sqrt(sigma));

	// Is the intersection point on the segment?
	if ((0.0f <= a) && (a <= (input.maxFraction * rr)))
	{
		a /= rr;
		output->fraction = a;
		output->normal = b2Normalize(s + a * r);
		return true;
	}

	return false;
}

void b2CircleShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	const auto p = transform.p + b2Mul(transform.q, m_p);
	aabb->lowerBound.Set(p.x - GetRadius(), p.y - GetRadius());
	aabb->upperBound.Set(p.x + GetRadius(), p.y + GetRadius());
}

void b2CircleShape::ComputeMass(b2MassData* massData, float32 density) const
{
	massData->mass = density * b2_pi * GetRadius() * GetRadius();
	massData->center = m_p;

	// inertia about the local origin
	massData->I = massData->mass * (0.5f * GetRadius() * GetRadius() + b2Dot(m_p, m_p));
}
