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

#ifndef COMPOUND_SHAPES_H
#define COMPOUND_SHAPES_H

namespace box2d {

// TODO_ERIN test joints on compounds.
class CompoundShapes : public Test
{
public:
	CompoundShapes()
	{
		{
			BodyDef bd;
			bd.position = Vec2(0.0f, 0.0f);
			Body* body = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(50.0f, 0.0f), Vec2(-50.0f, 0.0f));

			body->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			CircleShape circle1(float_t(0.5), Vec2(float_t(-0.5), float_t(0.5f)));

			CircleShape circle2(float_t(0.5), Vec2(float_t(0.5), float_t(0.5f)));

			for (int i = 0; i < 10; ++i)
			{
				float_t x = RandomFloat(-0.1f, 0.1f);
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(x + 5.0f, 1.05f + 2.5f * i);
				bd.angle = 1_rad * RandomFloat(-Pi, Pi);
				Body* body = m_world->CreateBody(bd);
				body->CreateFixture(FixtureDef{&circle1, 2.0f});
				body->CreateFixture(FixtureDef{&circle2, 0.0f});
			}
		}

		{
			const auto polygon1 = PolygonShape(0.25f, 0.5f);

			PolygonShape polygon2;
			SetAsBox(polygon2, 0.25f, 0.5f, Vec2(0.0f, -0.5f), 0.5_rad * Pi);

			for (int i = 0; i < 10; ++i)
			{
				float_t x = RandomFloat(-0.1f, 0.1f);
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(x - 5.0f, 1.05f + 2.5f * i);
				bd.angle = 1_rad * RandomFloat(-Pi, Pi);
				Body* body = m_world->CreateBody(bd);
				body->CreateFixture(FixtureDef{&polygon1, 2.0f});
				body->CreateFixture(FixtureDef{&polygon2, 2.0f});
			}
		}

		{
			Transformation xf1;
			xf1.q = UnitVec2{0.3524_rad * Pi};
			xf1.p = GetXAxis(xf1.q) * 1;

			const auto triangle1 = PolygonShape({
				Transform(Vec2(-1.0f, 0.0f), xf1),
				Transform(Vec2(1.0f, 0.0f), xf1),
				Transform(Vec2(0.0f, 0.5f), xf1)
			});

			Transformation xf2;
			xf2.q = UnitVec2{-0.3524_rad * Pi};
			xf2.p = -GetXAxis(xf2.q) * 1;

			const auto triangle2 = PolygonShape({
				Transform(Vec2(-1.0f, 0.0f), xf2),
				Transform(Vec2(1.0f, 0.0f), xf2),
				Transform(Vec2(0.0f, 0.5f), xf2)
			});
			
			for (int32 i = 0; i < 10; ++i)
			{
				float_t x = RandomFloat(-0.1f, 0.1f);
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(x, 2.05f + 2.5f * i);
				bd.angle = 0.0_rad;
				Body* body = m_world->CreateBody(bd);
				body->CreateFixture(FixtureDef{&triangle1, 2.0f});
				body->CreateFixture(FixtureDef{&triangle2, 2.0f});
			}
		}

		{
			const auto bottom = PolygonShape( 1.5f, 0.15f );

			PolygonShape left;
			SetAsBox(left, 0.15f, 2.7f, Vec2(-1.45f, 2.35f), 0.2_rad);

			PolygonShape right;
			SetAsBox(right, 0.15f, 2.7f, Vec2(1.45f, 2.35f), -0.2_rad);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2( 0.0f, 2.0f );
			Body* body = m_world->CreateBody(bd);
			body->CreateFixture(FixtureDef{&bottom, 4.0f});
			body->CreateFixture(FixtureDef{&left, 4.0f});
			body->CreateFixture(FixtureDef{&right, 4.0f});
		}
	}

	static Test* Create()
	{
		return new CompoundShapes;
	}
};

} // namespace box2d

#endif
