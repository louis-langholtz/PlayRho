/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <new>

using namespace box2d;

void b2EdgeShape::Set(const Vec2& v1, const Vec2& v2)
{
	m_vertex1 = v1;
	m_vertex2 = v2;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

b2Shape* b2EdgeShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2EdgeShape));
	return new (mem) b2EdgeShape(*this);
}

child_count_t b2EdgeShape::GetChildCount() const
{
	return 1;
}

bool b2EdgeShape::TestPoint(const b2Transform& xf, const Vec2& p) const
{
	BOX2D_NOT_USED(xf);
	BOX2D_NOT_USED(p);
	return false;
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
bool b2EdgeShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& xf, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	// Put the ray into the edge's frame of reference.
	const auto p1 = b2MulT(xf.q, input.p1 - xf.p);
	const auto p2 = b2MulT(xf.q, input.p2 - xf.p);
	const auto d = p2 - p1;

	const auto v1 = m_vertex1;
	const auto v2 = m_vertex2;
	const auto e = v2 - v1;
	const auto normal = b2Normalize(Vec2(e.y, -e.x));

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	const auto numerator = Dot(normal, v1 - p1);
	const auto denominator = Dot(normal, d);

	if (denominator == float_t{0})
	{
		return false;
	}

	const auto t = numerator / denominator;
	if ((t < float_t{0}) || (input.maxFraction < t))
	{
		return false;
	}

	const auto q = p1 + t * d;

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	const auto r = v2 - v1;
	const auto rr = r.LengthSquared();
	if (rr == float_t{0})
	{
		return false;
	}

	const auto s = Dot(q - v1, r) / rr;
	if ((s < float_t{0}) || (float_t(1) < s))
	{
		return false;
	}

	output->fraction = t;
	output->normal = (numerator > float_t{0})? -b2Mul(xf.q, normal): b2Mul(xf.q, normal);
	return true;
}

b2AABB b2EdgeShape::ComputeAABB(const b2Transform& xf, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	const auto v1 = b2Mul(xf, m_vertex1);
	const auto v2 = b2Mul(xf, m_vertex2);

	const auto lower = b2Min(v1, v2);
	const auto upper = b2Max(v1, v2);

	const auto r = Vec2{GetRadius(), GetRadius()};
	return b2AABB{lower - r, upper + r};
}

b2MassData b2EdgeShape::ComputeMass(float_t density) const
{
	BOX2D_NOT_USED(density);

	return b2MassData{float_t{0}, (m_vertex1 + m_vertex2) / float_t(2), float_t{0}};
}
