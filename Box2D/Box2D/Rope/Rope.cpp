/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

#include <Box2D/Rope/Rope.h>
#include <Box2D/Common/Draw.h>

using namespace box2d;

Rope::~Rope()
{
	free(m_ps);
	free(m_p0s);
	free(m_vs);
	free(m_ims);
	free(m_Ls);
	free(m_as);
}

void Rope::Initialize(const RopeDef* def)
{
	assert(def->count >= 3);
	m_count = def->count;
	m_ps = static_cast<Vec2*>(alloc(m_count * sizeof(Vec2)));
	m_p0s = static_cast<Vec2*>(alloc(m_count * sizeof(Vec2)));
	m_vs = static_cast<Vec2*>(alloc(m_count * sizeof(Vec2)));
	m_ims = static_cast<float_t*>(alloc(m_count * sizeof(float_t)));

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		m_ps[i] = def->vertices[i];
		m_p0s[i] = def->vertices[i];
		m_vs[i] = Vec2_zero;

		const auto m = def->masses[i];
		if (m > float_t{0})
		{
			m_ims[i] = float_t(1) / m;
		}
		else
		{
			m_ims[i] = float_t{0};
		}
	}

	const auto count2 = m_count - 1;
	const auto count3 = m_count - 2;
	m_Ls = static_cast<float_t*>(alloc(count2 * sizeof(float_t)));
	m_as = static_cast<float_t*>(alloc(count3 * sizeof(float_t)));

	for (auto i = decltype(count2){0}; i < count2; ++i)
	{
		const auto p1 = m_ps[i];
		const auto p2 = m_ps[i+1];
		m_Ls[i] = Distance(p1, p2);
	}

	for (auto i = decltype(count3){0}; i < count3; ++i)
	{
		const auto p1 = m_ps[i];
		const auto p2 = m_ps[i + 1];
		const auto p3 = m_ps[i + 2];

		const auto d1 = p2 - p1;
		const auto d2 = p3 - p2;

		const auto a = Cross(d1, d2);
		const auto b = Dot(d1, d2);

		m_as[i] = Atan2(a, b);
	}

	m_gravity = def->gravity;
	m_damping = def->damping;
	m_k2 = def->k2;
	m_k3 = def->k3;
}

void Rope::Step(float_t h, int32 iterations)
{
	if (h == 0.0)
	{
		return;
	}

	const auto d = std::exp(- h * m_damping);

	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		m_p0s[i] = m_ps[i];
		if (m_ims[i] > float_t{0})
		{
			m_vs[i] += h * m_gravity;
		}
		m_vs[i] *= d;
		m_ps[i] += h * m_vs[i];

	}

	for (auto i = decltype(iterations){0}; i < iterations; ++i)
	{
		SolveC2();
		SolveC3();
		SolveC2();
	}

	const auto inv_h = float_t{1} / h;
	for (auto i = decltype(m_count){0}; i < m_count; ++i)
	{
		m_vs[i] = inv_h * (m_ps[i] - m_p0s[i]);
	}
}

void Rope::SolveC2()
{
	const auto count2 = m_count - 1;

	for (auto i = decltype(count2){0}; i < count2; ++i)
	{
		auto p1 = m_ps[i];
		auto p2 = m_ps[i + 1];

		auto d = p2 - p1;
		const auto L = d.Normalize();

		const auto im1 = m_ims[i];
		const auto im2 = m_ims[i + 1];

		if (im1 + im2 == float_t{0})
		{
			continue;
		}

		const auto s1 = im1 / (im1 + im2);
		const auto s2 = im2 / (im1 + im2);

		p1 -= m_k2 * s1 * (m_Ls[i] - L) * d;
		p2 += m_k2 * s2 * (m_Ls[i] - L) * d;

		m_ps[i] = p1;
		m_ps[i + 1] = p2;
	}
}

void Rope::SetAngle(float_t angle)
{
	const auto count3 = m_count - 2;
	for (auto i = decltype(count3){0}; i < count3; ++i)
	{
		m_as[i] = angle;
	}
}

void Rope::SolveC3()
{
	const auto count3 = m_count - 2;

	for (auto i = decltype(count3){0}; i < count3; ++i)
	{
		auto p1 = m_ps[i];
		auto p2 = m_ps[i + 1];
		auto p3 = m_ps[i + 2];

		const auto m1 = m_ims[i];
		const auto m2 = m_ims[i + 1];
		const auto m3 = m_ims[i + 2];

		const auto d1 = p2 - p1;
		const auto d2 = p3 - p2;

		const auto L1sqr = d1.LengthSquared();
		const auto L2sqr = d2.LengthSquared();

		if (L1sqr * L2sqr == float_t{0})
		{
			continue;
		}

		const auto a = Cross(d1, d2);
		const auto b = Dot(d1, d2);

		auto angle = Atan2(a, b);

		const auto Jd1 = (-float_t(1) / L1sqr) * GetCcwPerpendicular(d1);
		const auto Jd2 = (float_t(1) / L2sqr) * GetCcwPerpendicular(d2);

		const auto J1 = -Jd1;
		const auto J2 = Jd1 - Jd2;
		const auto J3 = Jd2;

		auto mass = m1 * Dot(J1, J1) + m2 * Dot(J2, J2) + m3 * Dot(J3, J3);
		if (mass == float_t{0})
		{
			continue;
		}

		mass = float_t{1} / mass;

		auto C = angle - m_as[i];

		while (C > Pi)
		{
			angle -= 2 * Pi;
			C = angle - m_as[i];
		}

		while (C < -Pi)
		{
			angle += float_t{2} * Pi;
			C = angle - m_as[i];
		}

		const auto impulse = - m_k3 * mass * C;

		p1 += (m1 * impulse) * J1;
		p2 += (m2 * impulse) * J2;
		p3 += (m3 * impulse) * J3;

		m_ps[i] = p1;
		m_ps[i + 1] = p2;
		m_ps[i + 2] = p3;
	}
}

void Rope::Draw(box2d::Draw* draw) const
{
	const auto c = Color(float_t(0.4), float_t(0.5), float_t(0.7));

	for (auto i = decltype(m_count - 1){0}; i < m_count - 1; ++i)
	{
		draw->DrawSegment(m_ps[i], m_ps[i+1], c);
	}
}
