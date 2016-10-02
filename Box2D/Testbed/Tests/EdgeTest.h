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

#ifndef EDGE_TEST_H
#define EDGE_TEST_H

namespace box2d {

class EdgeTest : public Test
{
public:

	EdgeTest()
	{
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			Vec2 v1(-10.0f, 0.0f), v2(-7.0f, -2.0f), v3(-4.0f, 0.0f);
			Vec2 v4(0.0f, 0.0f), v5(4.0f, 0.0f), v6(7.0f, 2.0f), v7(10.0f, 0.0f);

			EdgeShape shape;

			shape.Set(v1, v2);
			shape.SetVertex3(v3);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});

			shape.Set(v2, v3);
			shape.SetVertex0(v1);
			shape.SetVertex3(v4);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});

			shape.Set(v3, v4);
			shape.SetVertex0(v2);
			shape.SetVertex3(v5);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});

			shape.Set(v4, v5);
			shape.SetVertex0(v3);
			shape.SetVertex3(v6);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});

			shape.Set(v5, v6);
			shape.SetVertex0(v4);
			shape.SetVertex3(v7);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});

			shape.Set(v6, v7);
			shape.SetVertex0(v5);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-0.5f, 0.6f);
			bd.allowSleep = false;
			Body* body = m_world->CreateBody(bd);

			CircleShape shape;
			shape.SetRadius(float_t(0.5));

			body->CreateFixture(FixtureDef{&shape, 1.0f});
		}

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(1.0f, 0.6f);
			bd.allowSleep = false;
			Body* body = m_world->CreateBody(bd);

			const auto shape = PolygonShape(0.5f, 0.5f);

			body->CreateFixture(FixtureDef{&shape, 1.0f});
		}
	}

	static Test* Create()
	{
		return new EdgeTest;
	}
};

} // namespace box2d

#endif
