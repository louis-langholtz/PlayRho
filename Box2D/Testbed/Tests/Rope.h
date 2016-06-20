/*
* Original work Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2016 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef ROPE_H
#define ROPE_H

///
class Rope : public Test
{
public:
	Rope()
	{
		const int32 N = 40;
		Vec2 vertices[N];
		float_t masses[N];

		for (int32 i = 0; i < N; ++i)
		{
			vertices[i] = Vec2(0.0f, 20.0f - 0.25f * i);
			masses[i] = 1.0f;
		}
		masses[0] = 0.0f;
		masses[1] = 0.0f;

		RopeDef def;
		def.vertices = vertices;
		def.count = N;
		def.gravity = Vec2(0.0f, -10.0f);
		def.masses = masses;
		def.damping = 0.1f;
		def.k2 = 1.0f;
		def.k3 = 0.5f;

		m_rope.Initialize(&def);

		m_angle = 0.0f;
		m_rope.SetAngle(m_angle);
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'q':
			m_angle = Max(-Pi, m_angle - 0.05f * Pi);
			m_rope.SetAngle(m_angle);
			break;

		case 'e':
			m_angle = Min(Pi, m_angle + 0.05f * Pi);
			m_rope.SetAngle(m_angle);
			break;
		}
	}

	void Step(Settings* settings)
	{
		float_t dt = settings->hz > 0.0f ? 1.0f / settings->hz : 0.0f;

		if (settings->pause && !settings->singleStep)
		{
			dt = 0.0f;
		}

		m_rope.Step(dt, 1);

		Test::Step(settings);

		m_rope.Draw(&m_debugDraw);

		m_debugDraw.DrawString(5, m_textLine, "Press (q,e) to adjust target angle");
		m_textLine += DRAW_STRING_NEW_LINE;
		m_debugDraw.DrawString(5, m_textLine, "Target angle = %g degrees", m_angle * 180.0f / Pi);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Rope;
	}

	Rope m_rope;
	float_t m_angle;
};

#endif
