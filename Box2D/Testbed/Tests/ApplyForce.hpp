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

#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

namespace box2d {

class ApplyForce : public Test
{
public:
	ApplyForce()
	{
		m_world->SetGravity(Vec2(RealNum{0}, RealNum{0}));

		const auto k_restitution = RealNum(0.4);

		Body* ground;
		{
			BodyDef bd;
			bd.position = Vec2(RealNum{0}, 20.0f);
			ground = m_world->CreateBody(bd);

			auto conf = EdgeShape::Conf{};
			conf.density = 0;
			conf.restitution = k_restitution;
			EdgeShape shape(conf);
			
			// Left vertical
			shape.Set(Vec2(-20.0f, -20.0f), Vec2(-20.0f, 20.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape));

			// Right vertical
			shape.Set(Vec2(20.0f, -20.0f), Vec2(20.0f, 20.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape));

			// Top horizontal
			shape.Set(Vec2(-20.0f, 20.0f), Vec2(20.0f, 20.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape));

			// Bottom horizontal
			shape.Set(Vec2(-20.0f, -20.0f), Vec2(20.0f, -20.0f));
			ground->CreateFixture(std::make_shared<EdgeShape>(shape));
		}

		{
			Transformation xf1;
			xf1.q = UnitVec2{0.3524f * Pi * Radian};
			xf1.p = Vec2{GetXAxis(xf1.q)};

			Vec2 vertices[3];
			vertices[0] = Transform(Vec2(-1.0f, RealNum{0}), xf1);
			vertices[1] = Transform(Vec2(1.0f, RealNum{0}), xf1);
			vertices[2] = Transform(Vec2(RealNum{0}, 0.5f), xf1);
			
			auto conf = PolygonShape::Conf{};
			
			conf.density = RealNum{4} * Kilogram / SquareMeter;
			const auto poly1 = PolygonShape(Span<const Vec2>(vertices, 3), conf);

			Transformation xf2;
			xf2.q = UnitVec2{-0.3524f * Pi * Radian};
			xf2.p = Vec2{-GetXAxis(xf2.q)};

			vertices[0] = Transform(Vec2(-1.0f, RealNum{0}), xf2);
			vertices[1] = Transform(Vec2(1.0f, RealNum{0}), xf2);
			vertices[2] = Transform(Vec2(RealNum{0}, 0.5f), xf2);

			conf.density = RealNum{2} * Kilogram / SquareMeter;
			const auto poly2 = PolygonShape(Span<const Vec2>(vertices, 3), conf);

			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.angularDamping = 2.0f;
			bd.linearDamping = 0.5f;

			bd.position = Vec2(0, 2);
			bd.angle = Pi * Radian;
			bd.allowSleep = false;
			m_body = m_world->CreateBody(bd);
			m_body->CreateFixture(std::make_shared<PolygonShape>(poly1));
			m_body->CreateFixture(std::make_shared<PolygonShape>(poly2));
		}

		{
			auto conf = PolygonShape::Conf{};
			conf.density = RealNum{1} * Kilogram / SquareMeter;
			conf.friction = 0.3f;
			const auto shape = std::make_shared<PolygonShape>(0.5f, 0.5f, conf);

			const auto gravity = 10.0f;
			
			for (auto i = 0; i < 10; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(RealNum{0}, 5.0f + 1.54f * i);
				const auto body = m_world->CreateBody(bd);

				body->CreateFixture(shape);

				const auto I = GetLocalInertia(*body);
				const auto mass = RealNum{GetMass(*body) / Kilogram};

				// For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
				const auto radius = Sqrt(2.0f * I / mass);

				FrictionJointDef jd;
				jd.localAnchorA = Vec2_zero;
				jd.localAnchorB = Vec2_zero;
				jd.bodyA = ground;
				jd.bodyB = body;
				jd.collideConnected = true;
				jd.maxForce = mass * gravity;
				jd.maxTorque = mass * radius * gravity;

				m_world->CreateJoint(jd);
			}
		}
	}

	void KeyboardDown(Key key)
	{
		switch (key)
		{
		case Key_W:
			{
				const auto f = GetWorldVector(*m_body, Vec2(RealNum{0}, -200.0f));
				const auto p = GetWorldPoint(*m_body, Vec2(RealNum{0}, 2.0f));
				box2d::ApplyForce(*m_body, f, p);
			}
			break;

		case Key_A:
			{
				ApplyTorque(*m_body, 50.0f);
			}
			break;

		case Key_D:
			{
				ApplyTorque(*m_body, -50.0f);
			}
			break;

		default:
			break;
		}
	}

	static Test* Create()
	{
		return new ApplyForce;
	}

	Body* m_body;
};

} // namespace box2d

#endif
