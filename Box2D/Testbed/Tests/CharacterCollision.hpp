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

#ifndef CHARACTER_COLLISION_H
#define CHARACTER_COLLISION_H

namespace box2d {

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on edge chains.
class CharacterCollision : public Test
{
public:
	CharacterCollision()
	{
		// Ground body
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			const auto shape = EdgeShape(Vec2(-20.0f, 0.0f), Vec2(20.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Collinear edges with no adjacency information.
		// This shows the problematic case where a box shape can hit
		// an internal vertex.
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-8.0f, 1.0f), Vec2(-6.0f, 1.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
			shape.Set(Vec2(-6.0f, 1.0f), Vec2(-4.0f, 1.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
			shape.Set(Vec2(-4.0f, 1.0f), Vec2(-2.0f, 1.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Chain shape
		{
			BodyDef bd;
			bd.angle = 0.25_rad * Pi;
			Body* ground = m_world->CreateBody(bd);

			Vec2 vs[4];
			vs[0] = Vec2(5.0f, 7.0f);
			vs[1] = Vec2(6.0f, 8.0f);
			vs[2] = Vec2(7.0f, 8.0f);
			vs[3] = Vec2(8.0f, 7.0f);
			ChainShape shape;
			shape.CreateChain(Span<const Vec2>(vs, 4));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Square tiles. This shows that adjacency shapes may
		// have non-smooth collision. There is no solution
		// to this problem.
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			PolygonShape shape;
			SetAsBox(shape, 1.0f, 1.0f, Vec2(4.0f, 3.0f), 0.0_rad);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
			SetAsBox(shape, 1.0f, 1.0f, Vec2(6.0f, 3.0f), 0.0_rad);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
			SetAsBox(shape, 1.0f, 1.0f, Vec2(8.0f, 3.0f), 0.0_rad);
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Square made from an edge loop. Collision should be smooth.
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			Vec2 vs[4];
			vs[0] = Vec2(-1.0f, 3.0f);
			vs[1] = Vec2(1.0f, 3.0f);
			vs[2] = Vec2(1.0f, 5.0f);
			vs[3] = Vec2(-1.0f, 5.0f);
			ChainShape shape;
			shape.CreateLoop(Span<const Vec2>(vs, 4));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Edge loop. Collision should be smooth.
		{
			BodyDef bd;
			bd.position = Vec2(-10.0f, 4.0f);
			Body* ground = m_world->CreateBody(bd);

			Vec2 vs[10];
			vs[0] = Vec2(0.0f, 0.0f);
			vs[1] = Vec2(6.0f, 0.0f);
			vs[2] = Vec2(6.0f, 2.0f);
			vs[3] = Vec2(4.0f, 1.0f);
			vs[4] = Vec2(2.0f, 2.0f);
			vs[5] = Vec2(0.0f, 2.0f);
			vs[6] = Vec2(-2.0f, 2.0f);
			vs[7] = Vec2(-4.0f, 3.0f);
			vs[8] = Vec2(-6.0f, 2.0f);
			vs[9] = Vec2(-6.0f, 0.0f);
			ChainShape shape;
			shape.CreateLoop(Span<const Vec2>(vs, 10));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		// Square character 1
		{
			BodyDef bd;
			bd.position = Vec2(-3.0f, 8.0f);
			bd.type = BodyType::Dynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			Body* body = m_world->CreateBody(bd);

			const auto shape = PolygonShape(0.5f, 0.5f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			body->CreateFixture(fd);
		}

		// Square character 2
		{
			BodyDef bd;
			bd.position = Vec2(-5.0f, 5.0f);
			bd.type = BodyType::Dynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			Body* body = m_world->CreateBody(bd);

			const auto shape = PolygonShape(0.25f, 0.25f);

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			body->CreateFixture(fd);
		}

		// Hexagon character
		{
			BodyDef bd;
			bd.position = Vec2(-5.0f, 8.0f);
			bd.type = BodyType::Dynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			Body* body = m_world->CreateBody(bd);

			float_t angle = 0.0f;
			float_t delta = Pi / 3.0f;
			Vec2 vertices[6];
			for (int32 i = 0; i < 6; ++i)
			{
				vertices[i] = Vec2(0.5f * cosf(angle), 0.5f * sinf(angle));
				angle += delta;
			}

			const auto shape = PolygonShape(Span<const Vec2>(vertices, 6));

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			body->CreateFixture(fd);
		}

		// Circle character
		{
			BodyDef bd;
			bd.position = Vec2(3.0f, 5.0f);
			bd.type = BodyType::Dynamic;
			bd.fixedRotation = true;
			bd.allowSleep = false;

			Body* body = m_world->CreateBody(bd);

			const auto shape = CircleShape(float_t(0.5));

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			body->CreateFixture(fd);
		}

		// Circle character
		{
			BodyDef bd;
			bd.position = Vec2(-7.0f, 6.0f);
			bd.type = BodyType::Dynamic;
			bd.allowSleep = false;

			m_character = m_world->CreateBody(bd);

			CircleShape shape;
			shape.SetRadius(float_t(0.25));

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0f;
			fd.friction = 1.0f;
			m_character->CreateFixture(fd);
		}
	}

	void Step(Settings& settings, Drawer& drawer) override
	{
		auto velocity = m_character->GetVelocity();
		velocity.v.x = -5.0f;
		m_character->SetVelocity(velocity);

		Test::Step(settings, drawer);
		drawer.DrawString(5, m_textLine, "This tests various character collision shapes.");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Limitation: square and hexagon can snag on aligned boxes.");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Feature: edge chains have smooth collision inside and out.");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new CharacterCollision;
	}

	Body* m_character;
};

} // namespace box2d

#endif
