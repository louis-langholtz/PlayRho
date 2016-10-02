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

#ifndef BRIDGE_H
#define BRIDGE_H

namespace box2d {

class Bridge : public Test
{
public:

	enum
	{
		e_count = 30
	};

	Bridge()
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
			const auto shape = PolygonShape(0.5f, 0.125f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 0.2f;

			Body* prevBody = ground;
			for (int32 i = 0; i < e_count; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(-14.5f + 1.0f * i, 5.0f);
				Body* body = m_world->CreateBody(bd);
				body->CreateFixture(fd);

				m_world->CreateJoint(RevoluteJointDef{prevBody, body, Vec2(-15.0f + 1.0f * i, 5.0f)});

				if (i == (e_count >> 1))
				{
					m_middle = body;
				}
				prevBody = body;
			}

			m_world->CreateJoint(RevoluteJointDef{prevBody, ground, Vec2(-15.0f + 1.0f * e_count, 5.0f)});
		}

		for (int32 i = 0; i < 2; ++i)
		{
			Vec2 vertices[3];
			vertices[0] = Vec2(-0.5f, 0.0f);
			vertices[1] = Vec2(0.5f, 0.0f);
			vertices[2] = Vec2(0.0f, 1.5f);

			PolygonShape shape;
			shape.Set(vertices, 3);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-8.0f + 8.0f * i, 12.0f);
			Body* body = m_world->CreateBody(bd);
			body->CreateFixture(fd);
		}

		for (int32 i = 0; i < 3; ++i)
		{
			CircleShape shape;
			shape.SetRadius(float_t(0.5));

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-6.0f + 6.0f * i, 10.0f);
			Body* body = m_world->CreateBody(bd);
			body->CreateFixture(fd);
		}
	}

	static Test* Create()
	{
		return new Bridge;
	}

	Body* m_middle;
};

} // namespace box2d

#endif
