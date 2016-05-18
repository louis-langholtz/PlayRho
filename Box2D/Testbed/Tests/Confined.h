/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
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
	
	enum
	{
		e_columnCount = 0,
		e_rowCount = 0
	};

	Confined()
	{
		{
			b2BodyDef bd;
			b2Body* ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.restitution = float_t(0.9);

			// Floor
			shape.Set(b2Vec2(-wall_length/2, 0.0f), b2Vec2(wall_length/2, 0.0f));
			ground->CreateFixture(&fd);

			// Left wall
			shape.Set(b2Vec2(-wall_length/2, 0.0f), b2Vec2(-wall_length/2, wall_length));
			ground->CreateFixture(&fd);

			// Right wall
			shape.Set(b2Vec2(wall_length/2, 0.0f), b2Vec2(wall_length/2, wall_length));
			ground->CreateFixture(&fd);

			// Roof
			shape.Set(b2Vec2(-wall_length/2, wall_length), b2Vec2(wall_length/2, wall_length));
			ground->CreateFixture(&fd);
		}

		float_t radius = 0.5f;
		b2CircleShape shape(radius, b2Vec2_zero);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0f;
		fd.friction = 0.1f;

		for (int32 j = 0; j < e_columnCount; ++j)
		{
			for (int i = 0; i < e_rowCount; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position = b2Vec2(-10.0f + (2.1f * j + 1.0f + 0.01f * i) * radius, (2.0f * i + 1.0f) * radius);
				b2Body* body = m_world->CreateBody(&bd);

				body->CreateFixture(&fd);
			}
		}

		m_world->SetGravity(b2Vec2(0.0f, 0.0f));
	}

	void CreateCircle()
	{
		constexpr auto radius = float_t(wall_length/10); // 2
		b2CircleShape shape(radius, b2Vec2_zero);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0f;
		fd.restitution = float_t(0.8);

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.bullet = m_bullet_mode;
		bd.position = b2Vec2(RandomFloat(-wall_length/2, +wall_length/2), RandomFloat(0, wall_length));
		//bd.allowSleep = false;
		b2Body* body = m_world->CreateBody(&bd);

		body->CreateFixture(&fd);
	}

	void CreateBox()
	{
		constexpr auto side_length = float_t(wall_length/5); // 4
		b2PolygonShape shape;
		shape.SetAsBox(side_length/2, side_length/2);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0f;
		fd.restitution = float_t(0.8);
		
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.bullet = m_bullet_mode;
		bd.position = b2Vec2(RandomFloat(-wall_length/2, +wall_length/2), RandomFloat(0, wall_length));
		auto* body = m_world->CreateBody(&bd);
		body->CreateFixture(&fd);
	}

	void ToggleBulletMode()
	{
		m_bullet_mode = !m_bullet_mode;
		for (auto& b: m_world->GetBodies())
		{
			if (b.GetType() == b2_dynamicBody)
			{
				b.SetBullet(m_bullet_mode);
			}
		}
	}

	void ImpartRandomImpulses()
	{
		for (auto& b: m_world->GetBodies())
		{
			if (b.GetType() == b2_dynamicBody)
			{
				const auto position = b.GetPosition();
				const auto angle_from_center = b2Atan2(position.y - wall_length/2, position.x);
				const auto opposite_angle = angle_from_center + b2_pi;
				const auto direction = opposite_angle;
				const auto magnitude = b2Sqrt(b2Square(wall_length) * 2) * b.GetMass() * 20;
				const auto impulse = b2Mul(b2Rot(direction), b2Vec2(magnitude, 0.0f));
				b.ApplyLinearImpulse(impulse, b.GetWorldCenter(), true);
			}
		}		
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_C:
			CreateCircle();
			break;
		case GLFW_KEY_B:
			CreateBox();
			break;
		case GLFW_KEY_I:
			ImpartRandomImpulses();
			break;
		case GLFW_KEY_PERIOD:
			ToggleBulletMode();
			break;
		}
	}

	void Step(Settings* settings)
	{
		bool sleeping = true;
		for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
		{
			if (b->GetType() != b2_dynamicBody)
			{
				continue;
			}

			if (b->IsAwake())
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

		Test::Step(settings);

		for (b2Body* b = m_world->GetBodyList(); b; b = b->GetNext())
		{
			if (b->GetType() != b2_dynamicBody)
			{
				continue;
			}

			b2Vec2 p = b->GetPosition();
			if (p.x <= -wall_length/2 || wall_length/2 <= p.x || p.y <= 0.0f || wall_length <= p.y)
			{
				p.x += 0.0f;
			}
		}

		g_debugDraw.DrawString(5, m_textLine, "Press 'c' to create a circle.");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "Press 'b' to create a box.");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "Press '.' to toggle bullet mode (currently %s).", m_bullet_mode? "on": "off");
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "Press 'i' to impart impulses.");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Confined;
	}
	
	bool m_bullet_mode = false;
};

} // namespace box2d

#endif
