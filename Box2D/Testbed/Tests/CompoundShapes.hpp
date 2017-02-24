/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
			const auto body = m_world->CreateBody(bd);
			body->CreateFixture(std::make_shared<EdgeShape>(Vec2(50.0f, 0.0f), Vec2(-50.0f, 0.0f)));
		}

		{
			const auto circle1 = std::make_shared<CircleShape>(0.5f, Vec2{-0.5f, 0.5f});
			const auto circle2 = std::make_shared<CircleShape>(0.5f, Vec2{0.5f, 0.5f});
			for (auto i = 0; i < 10; ++i)
			{
				const auto x = RandomFloat(-0.1f, 0.1f);
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(x + 5.0f, 1.05f + 2.5f * i);
				bd.angle = 1_rad * RandomFloat(-Pi, Pi);
				const auto body = m_world->CreateBody(bd);
				body->CreateFixture(circle1, FixtureDef{}.UseDensity(2));
				body->CreateFixture(circle2);
			}
		}

		{
			const auto polygon1 = std::make_shared<PolygonShape>(0.25f, 0.5f);
			auto polygon2 = std::make_shared<PolygonShape>();
			SetAsBox(*polygon2, 0.25f, 0.5f, Vec2(0.0f, -0.5f), 0.5_rad * Pi);
			for (int i = 0; i < 10; ++i)
			{
				const auto x = RandomFloat(-0.1f, 0.1f);
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(x - 5.0f, 1.05f + 2.5f * i);
				bd.angle = 1_rad * RandomFloat(-Pi, Pi);
				const auto body = m_world->CreateBody(bd);
				body->CreateFixture(polygon1, FixtureDef{}.UseDensity(2));
				body->CreateFixture(polygon2, FixtureDef{}.UseDensity(2));
			}
		}

		{
			Transformation xf1;
			xf1.q = UnitVec2{0.3524_rad * Pi};
			xf1.p = GetXAxis(xf1.q) * 1;

			auto triangle1 = std::make_shared<PolygonShape>();
			triangle1->Set(Span<const Vec2>{
				Transform(Vec2(-1.0f, 0.0f), xf1),
				Transform(Vec2(1.0f, 0.0f), xf1),
				Transform(Vec2(0.0f, 0.5f), xf1)
			});

			Transformation xf2;
			xf2.q = UnitVec2{-0.3524_rad * Pi};
			xf2.p = -GetXAxis(xf2.q) * 1;

			auto triangle2 = std::make_shared<PolygonShape>();
			triangle2->Set(Span<const Vec2>{
				Transform(Vec2(-1.0f, 0.0f), xf2),
				Transform(Vec2(1.0f, 0.0f), xf2),
				Transform(Vec2(0.0f, 0.5f), xf2)
			});
			
			for (auto i = 0; i < 10; ++i)
			{
				const auto x = RandomFloat(-0.1f, 0.1f);
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(x, 2.05f + 2.5f * i);
				bd.angle = 0.0_rad;
				const auto body = m_world->CreateBody(bd);
				body->CreateFixture(triangle1, FixtureDef{}.UseDensity(2));
				body->CreateFixture(triangle2, FixtureDef{}.UseDensity(2));
			}
		}

		{
			const auto bottom = std::make_shared<PolygonShape>(1.5f, 0.15f);

			auto left = std::make_shared<PolygonShape>();
			SetAsBox(*left, 0.15f, 2.7f, Vec2(-1.45f, 2.35f), 0.2_rad);

			auto right = std::make_shared<PolygonShape>();
			SetAsBox(*right, 0.15f, 2.7f, Vec2(1.45f, 2.35f), -0.2_rad);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2( 0.0f, 2.0f );
			const auto body = m_world->CreateBody(bd);
			body->CreateFixture(bottom, FixtureDef{}.UseDensity(4));
			body->CreateFixture(left, FixtureDef{}.UseDensity(4));
			body->CreateFixture(right, FixtureDef{}.UseDensity(4));
		}
	}

	static Test* Create()
	{
		return new CompoundShapes;
	}
};

} // namespace box2d

#endif
