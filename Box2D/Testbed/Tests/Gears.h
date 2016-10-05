/*
* Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef GEARS_H
#define GEARS_H

namespace box2d {

class Gears : public Test
{
public:
	Gears()
	{
		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(50.0f, 0.0f), Vec2(-50.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			CircleShape circle1;
			circle1.SetRadius(float_t(1.0));

			const auto box = PolygonShape(0.5f, 5.0f);

			CircleShape circle2;
			circle2.SetRadius(float_t(2.0));
			
			BodyDef bd1;
			bd1.type = BodyType::Static;
			bd1.position = Vec2(10.0f, 9.0f);
			Body* body1 = m_world->CreateBody(bd1);
			body1->CreateFixture(FixtureDef{&circle1, 5.0f});

			BodyDef bd2;
			bd2.type = BodyType::Dynamic;
			bd2.position = Vec2(10.0f, 8.0f);
			Body* body2 = m_world->CreateBody(bd2);
			body2->CreateFixture(FixtureDef{&box, 5.0f});

			BodyDef bd3;
			bd3.type = BodyType::Dynamic;
			bd3.position = Vec2(10.0f, 6.0f);
			Body* body3 = m_world->CreateBody(bd3);
			body3->CreateFixture(FixtureDef{&circle2, 5.0f});

			Joint* joint1 = m_world->CreateJoint(RevoluteJointDef{body2, body1, bd1.position});
			Joint* joint2 = m_world->CreateJoint(RevoluteJointDef{body2, body3, bd3.position});

			GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body3;
			jd4.joint1 = joint1;
			jd4.joint2 = joint2;
			jd4.ratio = circle2.GetRadius() / circle1.GetRadius();
			m_world->CreateJoint(jd4);
		}

		{
			CircleShape circle1;
			circle1.SetRadius(float_t(1));

			CircleShape circle2;
			circle2.SetRadius(float_t(2));
			
			const auto box = PolygonShape(0.5f, 5.0f);

			BodyDef bd1;
			bd1.type = BodyType::Dynamic;
			bd1.position = Vec2(-3.0f, 12.0f);
			Body* body1 = m_world->CreateBody(bd1);
			body1->CreateFixture(FixtureDef{&circle1, 5.0f});

			RevoluteJointDef jd1;
			jd1.bodyA = ground;
			jd1.bodyB = body1;
			jd1.localAnchorA = GetLocalPoint(*ground, bd1.position);
			jd1.localAnchorB = GetLocalPoint(*body1, bd1.position);
			jd1.referenceAngle = body1->GetAngle() - ground->GetAngle();
			m_joint1 = (RevoluteJoint*)m_world->CreateJoint(jd1);

			BodyDef bd2;
			bd2.type = BodyType::Dynamic;
			bd2.position = Vec2(0.0f, 12.0f);
			Body* body2 = m_world->CreateBody(bd2);
			body2->CreateFixture(FixtureDef{&circle2, 5.0f});

			RevoluteJointDef jd2(ground, body2, bd2.position);
			m_joint2 = (RevoluteJoint*)m_world->CreateJoint(jd2);

			BodyDef bd3;
			bd3.type = BodyType::Dynamic;
			bd3.position = Vec2(2.5f, 12.0f);
			Body* body3 = m_world->CreateBody(bd3);
			body3->CreateFixture(FixtureDef{&box, 5.0f});

			PrismaticJointDef jd3;
			jd3.Initialize(ground, body3, bd3.position, Vec2(0.0f, 1.0f));
			jd3.lowerTranslation = -5.0f;
			jd3.upperTranslation = 5.0f;
			jd3.enableLimit = true;

			m_joint3 = (PrismaticJoint*)m_world->CreateJoint(jd3);

			GearJointDef jd4;
			jd4.bodyA = body1;
			jd4.bodyB = body2;
			jd4.joint1 = m_joint1;
			jd4.joint2 = m_joint2;
			jd4.ratio = circle2.GetRadius() / circle1.GetRadius();
			m_joint4 = (GearJoint*)m_world->CreateJoint(jd4);

			GearJointDef jd5;
			jd5.bodyA = body2;
			jd5.bodyB = body3;
			jd5.joint1 = m_joint2;
			jd5.joint2 = m_joint3;
			jd5.ratio = -1.0f / circle2.GetRadius();
			m_joint5 = (GearJoint*)m_world->CreateJoint(jd5);
		}
	}

	void Step(Settings& settings, Drawer& drawer) override
	{
		Test::Step(settings, drawer);

		float_t ratio, value;
		
		ratio = m_joint4->GetRatio();
		value = m_joint1->GetJointAngle() + ratio * m_joint2->GetJointAngle();
		drawer.DrawString(5, m_textLine, "theta1 + %4.2f * theta2 = %4.2f", (float) ratio, (float) value);
		m_textLine += DRAW_STRING_NEW_LINE;

		ratio = m_joint5->GetRatio();
		value = m_joint2->GetJointAngle() + ratio * m_joint3->GetJointTranslation();
		drawer.DrawString(5, m_textLine, "theta2 + %4.2f * delta = %4.2f", (float) ratio, (float) value);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Gears;
	}

	RevoluteJoint* m_joint1;
	RevoluteJoint* m_joint2;
	PrismaticJoint* m_joint3;
	GearJoint* m_joint4;
	GearJoint* m_joint5;
};

} // namespace box2d

#endif
