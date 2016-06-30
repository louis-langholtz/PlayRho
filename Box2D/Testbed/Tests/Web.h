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

#ifndef WEB_H
#define WEB_H

namespace box2d {

// This tests distance joints, body destruction, and joint destruction.
class Web : public Test
{
public:
	Web()
	{
		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(&bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			PolygonShape shape;
			shape.SetAsBox(0.5f, 0.5f);

			BodyDef bd;
			bd.type = BodyType::Dynamic;

			bd.position = Vec2(-5.0f, 5.0f);
			m_bodies[0] = m_world->CreateBody(&bd);
			m_bodies[0]->CreateFixture(FixtureDef{&shape, 5.0f});

			bd.position = Vec2(5.0f, 5.0f);
			m_bodies[1] = m_world->CreateBody(&bd);
			m_bodies[1]->CreateFixture(FixtureDef{&shape, 5.0f});

			bd.position = Vec2(5.0f, 15.0f);
			m_bodies[2] = m_world->CreateBody(&bd);
			m_bodies[2]->CreateFixture(FixtureDef{&shape, 5.0f});

			bd.position = Vec2(-5.0f, 15.0f);
			m_bodies[3] = m_world->CreateBody(&bd);
			m_bodies[3]->CreateFixture(FixtureDef{&shape, 5.0f});

			DistanceJointDef jd;
			Vec2 p1, p2, d;

			jd.frequencyHz = 2.0f;
			jd.dampingRatio = 0.0f;

			jd.bodyA = ground;
			jd.bodyB = m_bodies[0];
			jd.localAnchorA = Vec2(-10.0f, 0.0f);
			jd.localAnchorB = Vec2(-0.5f, -0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[0] = m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[1];
			jd.localAnchorA = Vec2(10.0f, 0.0f);
			jd.localAnchorB = Vec2(0.5f, -0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[1] = m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[2];
			jd.localAnchorA = Vec2(10.0f, 20.0f);
			jd.localAnchorB = Vec2(0.5f, 0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[2] = m_world->CreateJoint(&jd);

			jd.bodyA = ground;
			jd.bodyB = m_bodies[3];
			jd.localAnchorA = Vec2(-10.0f, 20.0f);
			jd.localAnchorB = Vec2(-0.5f, 0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[3] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[0];
			jd.bodyB = m_bodies[1];
			jd.localAnchorA = Vec2(0.5f, 0.0f);
			jd.localAnchorB = Vec2(-0.5f, 0.0f);;
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[4] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[1];
			jd.bodyB = m_bodies[2];
			jd.localAnchorA = Vec2(0.0f, 0.5f);
			jd.localAnchorB = Vec2(0.0f, -0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[5] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[2];
			jd.bodyB = m_bodies[3];
			jd.localAnchorA = Vec2(-0.5f, 0.0f);
			jd.localAnchorB = Vec2(0.5f, 0.0f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[6] = m_world->CreateJoint(&jd);

			jd.bodyA = m_bodies[3];
			jd.bodyB = m_bodies[0];
			jd.localAnchorA = Vec2(0.0f, -0.5f);
			jd.localAnchorB = Vec2(0.0f, 0.5f);
			p1 = jd.bodyA->GetWorldPoint(jd.localAnchorA);
			p2 = jd.bodyB->GetWorldPoint(jd.localAnchorB);
			d = p2 - p1;
			jd.length = Length(d);
			m_joints[7] = m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_B:
			for (int32 i = 0; i < 4; ++i)
			{
				if (m_bodies[i])
				{
					m_world->DestroyBody(m_bodies[i]);
					m_bodies[i] = nullptr;
					break;
				}
			}
			break;

		case GLFW_KEY_J:
			for (int32 i = 0; i < 8; ++i)
			{
				if (m_joints[i])
				{
					m_world->DestroyJoint(m_joints[i]);
					m_joints[i] = nullptr;
					break;
				}
			}
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "This demonstrates a soft distance joint.");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "Press: (b) to delete a body, (j) to delete a joint");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	void JointDestroyed(Joint* joint)
	{
		for (int32 i = 0; i < 8; ++i)
		{
			if (m_joints[i] == joint)
			{
				m_joints[i] = nullptr;
				break;
			}
		}
	}

	static Test* Create()
	{
		return new Web;
	}

	Body* m_bodies[4];
	Joint* m_joints[8];
};

} // namespace box2d

#endif
