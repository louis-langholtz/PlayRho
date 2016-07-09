/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/EdgeShape.h>
#include <new>

using namespace box2d;

void EdgeShape::Set(const Vec2& v1, const Vec2& v2)
{
	m_vertex1 = v1;
	m_vertex2 = v2;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

Shape* EdgeShape::Clone(BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(EdgeShape));
	return new (mem) EdgeShape(*this);
}

child_count_t EdgeShape::GetChildCount() const
{
	return 1;
}

bool EdgeShape::TestPoint(const Transformation& xf, const Vec2& p) const
{
	BOX2D_NOT_USED(xf);
	BOX2D_NOT_USED(p);
	return false;
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
bool EdgeShape::RayCast(RayCastOutput* output, const RayCastInput& input,
							const Transformation& xf, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	// Put the ray into the edge's frame of reference.
	const auto p1 = InverseRotate(input.p1 - xf.p, xf.q);
	const auto p2 = InverseRotate(input.p2 - xf.p, xf.q);
	const auto d = p2 - p1;

	const auto v1 = m_vertex1;
	const auto v2 = m_vertex2;
	const auto e = v2 - v1;
	const auto normal = GetUnitVector(GetForwardPerpendicular(e));

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
	if ((t < float_t{0}) || (t > input.maxFraction))
	{
		return false;
	}

	const auto q = p1 + t * d;

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	const auto r = v2 - v1;
	const auto rr = LengthSquared(r);
	if (rr == float_t{0})
	{
		return false;
	}

	const auto s = Dot(q - v1, r) / rr;
	if ((s < float_t{0}) || (float_t{1} < s))
	{
		return false;
	}

	output->fraction = t;
	output->normal = (numerator > float_t{0})? -Rotate(normal, xf.q): Rotate(normal, xf.q);
	return true;
}

AABB EdgeShape::ComputeAABB(const Transformation& xf, child_count_t childIndex) const
{
	BOX2D_NOT_USED(childIndex);

	const auto v1 = Transform(m_vertex1, xf);
	const auto v2 = Transform(m_vertex2, xf);

	const auto lower = Min(v1, v2);
	const auto upper = Max(v1, v2);

	const auto r = Vec2{GetRadius(), GetRadius()};
	return AABB{lower - r, upper + r};
}

MassData EdgeShape::ComputeMass(float_t density) const
{
	BOX2D_NOT_USED(density);

	return MassData{float_t{0}, (m_vertex1 + m_vertex2) / float_t(2), float_t{0}};
}
