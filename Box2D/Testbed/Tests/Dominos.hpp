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

#ifndef DOMINOS_H
#define DOMINOS_H

namespace box2d {

class Dominos : public Test
{
public:

	Dominos()
	{
		Body* b1;
		{
			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));

			BodyDef bd;
			b1 = m_world->CreateBody(bd);
			b1->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			const auto shape = PolygonShape(6.0f, 0.25f);

			BodyDef bd;
			bd.position = Vec2(-1.5f, 10.0f);
			Body* ground = m_world->CreateBody(bd);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			const auto shape = PolygonShape(0.1f, 1.0f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 0.1f;

			for (int i = 0; i < 10; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(-6.0f + 1.0f * i, 11.25f);
				Body* body = m_world->CreateBody(bd);
				body->CreateFixture(fd);
			}
		}

		{
			PolygonShape shape;
			SetAsBox(shape, 7.0f, 0.25f, Vec2_zero, 0.3_rad);

			BodyDef bd;
			bd.position = Vec2(1.0f, 6.0f);
			Body* ground = m_world->CreateBody(bd);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		Body* b2;
		{
			const auto shape = PolygonShape(0.25f, 1.5f);

			BodyDef bd;
			bd.position = Vec2(-7.0f, 4.0f);
			b2 = m_world->CreateBody(bd);
			b2->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		Body* b3;
		{
			const auto shape = PolygonShape(6.0f, 0.125f);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-0.9f, 1.0f);
			bd.angle = -0.15_rad;

			b3 = m_world->CreateBody(bd);
			b3->CreateFixture(FixtureDef{&shape, 10.0f});
		}

		m_world->CreateJoint(RevoluteJointDef{b1, b3, Vec2(-2.0f, 1.0f), true});

		Body* b4;
		{
			const auto shape = PolygonShape(0.25f, 0.25f);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-10.0f, 15.0f);
			b4 = m_world->CreateBody(bd);
			b4->CreateFixture(FixtureDef{&shape, 10.0f});
		}

		m_world->CreateJoint(RevoluteJointDef{b2, b4, Vec2(-7.0f, 15.0f), true});

		Body* b5;
		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(6.5f, 3.0f);
			b5 = m_world->CreateBody(bd);

			PolygonShape shape;
			FixtureDef fd;

			fd.shape = &shape;
			fd.density = 10.0f;
			fd.friction = 0.1f;

			SetAsBox(shape, 1.0f, 0.1f, Vec2(0.0f, -0.9f), 0.0_rad);
			b5->CreateFixture(fd);

			SetAsBox(shape, 0.1f, 1.0f, Vec2(-0.9f, 0.0f), 0.0_rad);
			b5->CreateFixture(fd);

			SetAsBox(shape, 0.1f, 1.0f, Vec2(0.9f, 0.0f), 0.0_rad);
			b5->CreateFixture(fd);
		}

		m_world->CreateJoint(RevoluteJointDef{b1, b5, Vec2(6.0f, 2.0f), true});

		Body* b6;
		{
			const auto shape = PolygonShape(1.0f, 0.1f);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(6.5f, 4.1f);
			b6 = m_world->CreateBody(bd);
			b6->CreateFixture(FixtureDef{&shape, 30.0f});
		}

		m_world->CreateJoint(RevoluteJointDef{b5, b6, Vec2(7.5f, 4.0f), true});

		Body* b7;
		{
			const auto shape = PolygonShape(0.1f, 1.0f);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(7.4f, 1.0f);

			b7 = m_world->CreateBody(bd);
			b7->CreateFixture(FixtureDef{&shape, 10.0f});
		}

		DistanceJointDef djd;
		djd.bodyA = b3;
		djd.bodyB = b7;
		djd.localAnchorA = Vec2(6.0f, 0.0f);
		djd.localAnchorB = Vec2(0.0f, -1.0f);
		Vec2 d = GetWorldPoint(*djd.bodyB, djd.localAnchorB) - GetWorldPoint(*djd.bodyA, djd.localAnchorA);
		djd.length = GetLength(d);
		m_world->CreateJoint(djd);

		{
			float_t radius = 0.2f;

			CircleShape shape;
			shape.SetRadius(radius);

			for (int32 i = 0; i < 4; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(5.9f + 2.0f * radius * i, 2.4f);
				Body* body = m_world->CreateBody(bd);
				body->CreateFixture(FixtureDef{&shape, 10.0f});
			}
		}
	}

	static Test* Create()
	{
		return new Dominos;
	}
};

} // namespace box2d

#endif
