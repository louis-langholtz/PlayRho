/*
* Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#include <Box2D/Rope/Rope.hpp>

using namespace box2d;

Rope::~Rope()
{
    Free(m_ps);
    Free(m_p0s);
    Free(m_vs);
    Free(m_ims);
    Free(m_Ls);
    Free(m_as);
}

void Rope::Initialize(const RopeDef* def)
{
    assert(def->count >= 3);
    m_count = def->count;
    m_ps = alloc<Vec2>(m_count);
    m_p0s = alloc<Vec2>(m_count);
    m_vs = alloc<Vec2>(m_count);
    m_ims = alloc<RealNum>(m_count);

    for (auto i = decltype(m_count){0}; i < m_count; ++i)
    {
        m_ps[i] = def->vertices[i];
        m_p0s[i] = def->vertices[i];
        m_vs[i] = Vec2_zero;

        const auto m = def->masses[i];
        if (m > RealNum{0})
        {
            m_ims[i] = RealNum{1} / m;
        }
        else
        {
            m_ims[i] = RealNum{0};
        }
    }

    const auto count2 = m_count - 1;
    const auto count3 = m_count - 2;
    m_Ls = alloc<RealNum>(count2);
    m_as = alloc<Angle>(count3);

    for (auto i = decltype(count2){0}; i < count2; ++i)
    {
        const auto p1 = m_ps[i];
        const auto p2 = m_ps[i+1];
        m_Ls[i] = Sqrt(GetLengthSquared(p1 - p2));
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

void Rope::Step(RealNum h, int iterations)
{
    if (h == 0.0f)
    {
        return;
    }

    const auto d = std::exp(-h * m_damping);

    for (auto i = decltype(m_count){0}; i < m_count; ++i)
    {
        m_p0s[i] = m_ps[i];
        if (m_ims[i] > RealNum{0})
        {
            m_vs[i] += h * m_gravity;
        }
        m_vs[i] *= RealNum{d};
        m_ps[i] += h * m_vs[i];
    }

    for (auto i = decltype(iterations){0}; i < iterations; ++i)
    {
        SolveC2();
        SolveC3();
        SolveC2();
    }

    const auto inv_h = RealNum{1} / h;
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
        const auto L = Normalize(d);

        const auto im1 = m_ims[i];
        const auto im2 = m_ims[i + 1];

        if (im1 + im2 == RealNum{0})
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

void Rope::SetAngle(Angle angle)
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

        const auto L1sqr = GetLengthSquared(d1);
        const auto L2sqr = GetLengthSquared(d2);

        if (L1sqr * L2sqr == RealNum{0})
        {
            continue;
        }

        const auto a = Cross(d1, d2);
        const auto b = Dot(d1, d2);

        auto angle = Atan2(a, b);

        const auto Jd1 = (-RealNum{1} / L1sqr) * GetRevPerpendicular(d1);
        const auto Jd2 = (RealNum{1} / L2sqr) * GetRevPerpendicular(d2);

        const auto J1 = -Jd1;
        const auto J2 = Jd1 - Jd2;
        const auto J3 = Jd2;

        auto mass = m1 * Dot(J1, J1) + m2 * Dot(J2, J2) + m3 * Dot(J3, J3);
        if (mass == RealNum{0})
        {
            continue;
        }

        mass = RealNum{1} / mass;

        auto C = angle - m_as[i];

        while (C > Pi * Radian)
        {
            angle -= Pi * RealNum{2} * Radian;
            C = angle - m_as[i];
        }

        while (C < -Pi * Radian)
        {
            angle += Pi * RealNum{2} * Radian;
            C = angle - m_as[i];
        }

        const auto impulse = - m_k3 * mass * StripUnit(C);

        p1 += (m1 * impulse) * J1;
        p2 += (m2 * impulse) * J2;
        p3 += (m3 * impulse) * J3;

        m_ps[i] = p1;
        m_ps[i + 1] = p2;
        m_ps[i + 2] = p3;
    }
}
