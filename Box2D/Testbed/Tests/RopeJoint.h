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

#ifndef ROPE_JOINT_H
#define ROPE_JOINT_H

namespace box2d {

/// This test shows how a rope joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the rope joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the rope joint you can see that the Box2D solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
class RopeJointTest : public Test
{
public:
	RopeJointTest()
	{
		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			PolygonShape shape;
			shape.SetAsBox(0.5f, 0.125f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 0.2f;
			fd.filter.categoryBits = 0x0001;
			fd.filter.maskBits = 0xFFFF & ~0x0002;

			const int32 N = 10;
			const float_t y = 15.0f;
			m_ropeDef.localAnchorA = Vec2(0.0f, y);

			Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(0.5f + 1.0f * i, y);
				if (i == N - 1)
				{
					shape.SetAsBox(1.5f, 1.5f);
					fd.density = 100.0f;
					fd.filter.categoryBits = 0x0002;
					bd.position = Vec2(1.0f * i, y);
					bd.angularDamping = 0.4f;
				}

				Body* body = m_world->CreateBody(bd);

				body->CreateFixture(fd);

				Vec2 anchor(float_t(i), y);
				RevoluteJointDef jd{prevBody, body, anchor};
				jd.collideConnected = false;
				m_world->CreateJoint(jd);

				prevBody = body;
			}

			m_ropeDef.localAnchorB = Vec2_zero;

			float_t extraLength = 0.01f;
			m_ropeDef.maxLength = N - 1.0f + extraLength;
			m_ropeDef.bodyB = prevBody;
		}

		{
			m_ropeDef.bodyA = ground;
			m_rope = m_world->CreateJoint(m_ropeDef);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_J:
			if (m_rope)
			{
				m_world->Destroy(m_rope);
				m_rope = nullptr;
			}
			else
			{
				m_rope = m_world->CreateJoint(m_ropeDef);
			}
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Press (j) to toggle the rope joint.");
		m_textLine += DRAW_STRING_NEW_LINE;
		if (m_rope)
		{
			g_debugDraw.DrawString(5, m_textLine, "Rope ON");
		}
		else
		{
			g_debugDraw.DrawString(5, m_textLine, "Rope OFF");
		}
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new RopeJointTest;
	}

	RopeJointDef m_ropeDef;
	Joint* m_rope;
};

} // namespace box2d

#endif
