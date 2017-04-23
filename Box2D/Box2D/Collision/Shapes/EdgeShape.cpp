/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/EdgeShape.hpp>
#include <Box2D/Collision/RayCastInput.hpp>

using namespace box2d;

MassData EdgeShape::GetMassData() const noexcept
{
	return ::GetMassData(GetVertexRadius(), GetDensity(), GetVertex1(), GetVertex2());
}

void EdgeShape::Set(const Length2D v1, const Length2D v2)
{
	m_vertex1 = v1;
	m_vertex2 = v2;

	m_normal1 = GetUnitVector(GetFwdPerpendicular(v2 - v1));
	m_normal2 = -m_normal1;
}

bool EdgeShape::TestPoint(const Transformation& xf, const Length2D p) const noexcept
{
	NOT_USED(xf);
	NOT_USED(p);
	return false;
}

RayCastOutput EdgeShape::RayCast(const RayCastInput& input, const Transformation& xf,
								   child_count_t childIndex) const noexcept
{
	NOT_USED(childIndex);
	
	// p = p1 + t * d
	// v = v1 + s * e
	// p1 + t * d = v1 + s * e
	// s * e - t * d = p1 - v1
	
	// Put the ray into the edge's frame of reference.
	const auto d1 = input.p1 - xf.p;
	const auto p1 = InverseRotate(StripUnits(d1), xf.q);
	const auto d2 = input.p2 - xf.p;
	const auto p2 = InverseRotate(StripUnits(d2), xf.q);
	const auto d = p2 - p1;
	
	const auto v1 = GetVertex1();
	const auto v2 = GetVertex2();
	const auto e = v2 - v1;
	const auto eUnitless = StripUnits(e);
	const auto normal = GetUnitVector(GetFwdPerpendicular(eUnitless), UnitVec2::GetZero());
	
	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	const auto v1p1 = v1 - p1 * Meter;
	const auto numerator = Dot(normal, StripUnits(v1p1));
	const auto denominator = Dot(normal, d);
	
	if (denominator == 0)
	{
		return RayCastOutput{};
	}
	
	const auto t = numerator / denominator;
	if ((t < 0) || (t > input.maxFraction))
	{
		return RayCastOutput{};
	}
	
	const auto q = p1 + t * d;
	
	// q = v1 + s * e
	// s = dot(q - v1, e) / dot(e, e)
	const auto ee = GetLengthSquared(eUnitless);
	if (ee == 0)
	{
		return RayCastOutput{};
	}
	
	const auto qv1 = q * Meter - v1;
	const auto s = Dot(StripUnits(qv1), eUnitless) / ee;
	if ((s < 0) || (s > 1))
	{
		return RayCastOutput{};
	}
	
	const auto n = (numerator > 0)? -Rotate(normal, xf.q): Rotate(normal, xf.q);
	return RayCastOutput{n, t};
}
