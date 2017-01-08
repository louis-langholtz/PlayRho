/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#ifndef CONFINED_H
#define CONFINED_H

namespace box2d {

class Confined : public Test
{
public:
	static constexpr auto wall_length = float_t(0.05); // 20
	static constexpr auto vertexRadiusIncrement = wall_length / 80;
	
	enum
	{
		e_columnCount = 0,
		e_rowCount = 0
	};

	Confined()
	{
		m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);

		float_t radius = 0.5f;
		CircleShape shape(radius, Vec2_zero);

		FixtureDef fd;
		fd.density = 1.0f;
		fd.friction = 0.1f;

		for (int32 j = 0; j < e_columnCount; ++j)
		{
			for (int i = 0; i < e_rowCount; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(-10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
				Body* body = m_world->CreateBody(bd);

				body->CreateFixture(&shape, fd);
			}
		}

		m_world->SetGravity(Vec2(0.0f, 0.0f));
	}

	Body* CreateEnclosure(float_t vertexRadius, float_t wallLength)
	{
		BodyDef bd;
		const auto ground = m_world->CreateBody(bd);
		
		auto shape = EdgeShape{vertexRadius};
		//PolygonShape shape;
		
		FixtureDef fd;
		fd.restitution = float_t(0); // originally 0.9
		
		const auto btmLeft = Vec2(-wallLength/2, 0.0f);
		const auto btmRight = Vec2(wallLength/2, 0.0f);
		const auto topLeft = Vec2(-wallLength/2, wallLength);
		const auto topRight = Vec2(wallLength/2, wallLength);
		
		// Floor
		shape.Set(btmLeft, btmRight);
		//shape.Set(Span<const Vec2>{btmLeft, btmRight});
		ground->CreateFixture(&shape, fd);
		
		// Left wall
		shape.Set(btmLeft, topLeft);
		//shape.Set(Span<const Vec2>{btmLeft, topLeft});
		ground->CreateFixture(&shape, fd);
		
		// Right wall
		shape.Set(btmRight, topRight);
		//shape.Set(Span<const Vec2>{btmRight, topRight});
		ground->CreateFixture(&shape, fd);
		
		// Roof
		shape.Set(topLeft, topRight);
		//shape.Set(Span<const Vec2>{topLeft, topRight});
		ground->CreateFixture(&shape, fd);
		
		return ground;
	}
	
	void CreateCircle()
	{
		constexpr auto radius = float_t(wall_length/10); // 2
		const auto shape = CircleShape(radius, Vec2_zero);

		FixtureDef fd;
		fd.density = 1.0f;
		fd.restitution = float_t(0.8);

		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.bullet = m_bullet_mode;
		bd.position = Vec2(RandomFloat(-wall_length/2, +wall_length/2), RandomFloat(0, wall_length));
		//bd.allowSleep = false;
		Body* body = m_world->CreateBody(bd);

		body->CreateFixture(&shape, fd);
	}

	void CreateBox()
	{
		constexpr auto side_length = float_t(wall_length/5); // 4
		const auto shape = PolygonShape(side_length/2, side_length/2);

		FixtureDef fd;
		fd.density = 1.0f;
		fd.restitution = float_t(0); // originally 0.8
		
		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.bullet = m_bullet_mode;
		bd.position = Vec2(RandomFloat(-wall_length/2, +wall_length/2), RandomFloat(0, wall_length));
		auto* body = m_world->CreateBody(bd);
		body->CreateFixture(&shape, fd);
	}

	void ToggleBulletMode()
	{
		m_bullet_mode = !m_bullet_mode;
		for (auto& b: m_world->GetBodies())
		{
			if (b.GetType() == BodyType::Dynamic)
			{
				b.SetBullet(m_bullet_mode);
			}
		}
	}

	void ImpartRandomImpulses()
	{
		for (auto& b: m_world->GetBodies())
		{
			if (b.GetType() == BodyType::Dynamic)
			{
				const auto position = b.GetLocation();
				const auto angle_from_center = Atan2(position.y - wall_length/2, position.x);
				const auto opposite_angle = angle_from_center + Pi;
				const auto direction = opposite_angle;
				const auto magnitude = Sqrt(Square(wall_length) * 2) * GetMass(b) * 20;
				const auto impulse = Rotate(Vec2(magnitude, 0.0f), UnitVec2{1_rad * direction});
				ApplyLinearImpulse(b, impulse, b.GetWorldCenter());
			}
		}		
	}

	void Keyboard(Key key) override
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

	void PreStep(const Settings& settings, Drawer& drawer) override
	{
		auto sleeping = true;
		for (auto& b: m_world->GetBodies())
		{
			if (b.GetType() != BodyType::Dynamic)
			{
				continue;
			}

			if (b.IsAwake())
			{
				sleeping = false;
			}
		}

		if (m_stepCount == 180)
		{
			m_stepCount += 0;
		}

		//if (sleeping)
		//{
		//	CreateCircle();
		//}
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		auto i = 0;
		for (auto& b: m_world->GetBodies())
		{
			++i;
			if (b.GetType() != BodyType::Dynamic)
			{
				continue;
			}
			
			const auto location = b.GetLocation();
			drawer.DrawString(location, "B%d", i);
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
	float_t m_enclosureVertexRadius = vertexRadiusIncrement;
	Body* m_enclosure = nullptr;
};

} // namespace box2d

#endif
