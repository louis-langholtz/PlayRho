/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#ifndef CONFINED_H
#define CONFINED_H

namespace box2d {

class Confined : public Test
{
public:
	const RealNum wall_length = RealNum(0.1f); // DefaultLinearSlop * 1000
	const RealNum vertexRadiusIncrement = wall_length / 40;
	
	enum
	{
		e_columnCount = 0,
		e_rowCount = 0
	};

	Confined()
	{
		m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);

		const auto radius = 0.5f;
		auto conf = CircleShape::Conf{};
		conf.vertexRadius = radius;
		conf.density = 1.0f;
		conf.friction = 0.1f;
		const auto shape = std::make_shared<CircleShape>(conf);

		for (auto j = 0; j < e_columnCount; ++j)
		{
			for (auto i = 0; i < e_rowCount; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(-10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
				const auto body = m_world->CreateBody(bd);
				body->CreateFixture(shape);
			}
		}

		m_world->SetGravity(Vec2(0.0f, 0.0f));
	}

	Body* CreateEnclosure(RealNum vertexRadius, RealNum wallLength)
	{
		const auto ground = m_world->CreateBody();
		
		auto conf = EdgeShape::Conf{};
		conf.restitution = 0; // originally 0.9
		conf.vertexRadius = vertexRadius;
		auto shape = EdgeShape{conf};
		//PolygonShape shape;
		
		const auto btmLeft = Vec2(-wallLength/2, 0.0f);
		const auto btmRight = Vec2(wallLength/2, 0.0f);
		const auto topLeft = Vec2(-wallLength/2, wallLength);
		const auto topRight = Vec2(wallLength/2, wallLength);
		
		// Floor
		shape.Set(btmLeft, btmRight);
		//shape.Set(Span<const Vec2>{btmLeft, btmRight});
		ground->CreateFixture(std::make_shared<EdgeShape>(shape));
		
		// Left wall
		shape.Set(btmLeft, topLeft);
		//shape.Set(Span<const Vec2>{btmLeft, topLeft});
		ground->CreateFixture(std::make_shared<EdgeShape>(shape));
		
		// Right wall
		shape.Set(btmRight, topRight);
		//shape.Set(Span<const Vec2>{btmRight, topRight});
		ground->CreateFixture(std::make_shared<EdgeShape>(shape));
		
		// Roof
		shape.Set(topLeft, topRight);
		//shape.Set(Span<const Vec2>{topLeft, topRight});
		ground->CreateFixture(std::make_shared<EdgeShape>(shape));
		
		return ground;
	}
	
	void CreateCircle()
	{
		const auto radius = RealNum(wall_length/10); // 2

		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.bullet = m_bullet_mode;
		bd.position = Vec2(RandomFloat(-wall_length/2, +wall_length/2), RandomFloat(0, wall_length));
		bd.userData = reinterpret_cast<void*>(m_sequence);
		//bd.allowSleep = false;

		const auto body = m_world->CreateBody(bd);
		
		auto conf = CircleShape::Conf{};
		conf.density = 1.0f;
		conf.restitution = 0.8f;
		conf.vertexRadius = radius;
		body->CreateFixture(std::make_shared<CircleShape>(conf));

		++m_sequence;
	}

	void CreateBox()
	{
		const auto side_length = RealNum(wall_length/5); // 4

		auto conf = PolygonShape::Conf{};
		conf.density = 1.0f;
		conf.restitution = 0; // originally 0.8
		
		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.bullet = m_bullet_mode;
		bd.position = Vec2(RandomFloat(-wall_length/2, +wall_length/2), RandomFloat(0, wall_length));
		bd.userData = reinterpret_cast<void*>(m_sequence);
		const auto body = m_world->CreateBody(bd);
		body->CreateFixture(std::make_shared<PolygonShape>(side_length/2, side_length/2, conf));

		++m_sequence;
	}

	void ToggleBulletMode()
	{
		m_bullet_mode = !m_bullet_mode;
		for (auto& b: m_world->GetBodies())
		{
			if (b->GetType() == BodyType::Dynamic)
			{
				b->SetBullet(m_bullet_mode);
			}
		}
	}

	void ImpartRandomImpulses()
	{
		for (auto& b: m_world->GetBodies())
		{
			if (b->GetType() == BodyType::Dynamic)
			{
				const auto position = b->GetLocation();
				const auto angle_from_center = Atan2(position.y - wall_length/2, position.x);
				const auto opposite_angle = angle_from_center + Pi;
				const auto direction = opposite_angle;
				const auto magnitude = Sqrt(Square(wall_length) * 2) * GetMass(*b) * 20;
				const auto impulse = Rotate(Vec2(magnitude, 0.0f), UnitVec2{1_rad * direction});
				ApplyLinearImpulse(*b, impulse, b->GetWorldCenter());
			}
		}		
	}

	void KeyboardDown(Key key) override
	{
		switch (key)
		{
		case Key_C:
			CreateCircle();
			break;
		case Key_B:
			CreateBox();
			break;
		case Key_I:
			ImpartRandomImpulses();
			break;
		case Key_Period:
			ToggleBulletMode();
			break;
		case Key_Add:
			m_world->Destroy(m_enclosure);
			m_enclosureVertexRadius += vertexRadiusIncrement;
			m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
			break;
		case Key_Subtract:
			m_world->Destroy(m_enclosure);
			m_enclosureVertexRadius -= vertexRadiusIncrement;
			if (m_enclosureVertexRadius < 0)
			{
				m_enclosureVertexRadius = 0;
			}
			m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
			break;
		default:
			break;
		}
	}

	void PreStep(const Settings&, Drawer&) override
	{
		auto sleeping = true;
		for (auto& b: m_world->GetBodies())
		{
			if (b->GetType() != BodyType::Dynamic)
			{
				continue;
			}

			if (b->IsAwake())
			{
				sleeping = false;
			}
		}

		//if (sleeping)
		//{
		//	CreateCircle();
		//}
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		auto i = 0;
		for (auto& b: m_world->GetBodies())
		{
			++i;
			if (b->GetType() != BodyType::Dynamic)
			{
				continue;
			}
			
			const auto location = b->GetLocation();
			const auto userData = b->GetUserData();
			drawer.DrawString(location, "B%d", reinterpret_cast<decltype(m_sequence)>(userData));
		}
		
		drawer.DrawString(5, m_textLine, "Press 'c' to create a circle.");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Press 'b' to create a box.");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Press '.' to toggle bullet mode (currently %s).", m_bullet_mode? "on": "off");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Press 'i' to impart impulses.");
		m_textLine += DRAW_STRING_NEW_LINE;
	}
	
	static Test* Create()
	{
		return new Confined;
	}
	
	bool m_bullet_mode = false;
	RealNum m_enclosureVertexRadius = vertexRadiusIncrement;
	Body* m_enclosure = nullptr;
	size_t m_sequence = 0;
};

} // namespace box2d

#endif
