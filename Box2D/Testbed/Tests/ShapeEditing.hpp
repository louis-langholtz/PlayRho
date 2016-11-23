/*
* Original work Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
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

#ifndef SHAPE_EDITING_H
#define SHAPE_EDITING_H

namespace box2d {

class ShapeEditing : public Test
{
public:

	ShapeEditing()
	{
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		BodyDef bd;
		bd.type = BodyType::Dynamic;
		bd.position = Vec2(0.0f, 10.0f);
		m_body = m_world->CreateBody(bd);

		PolygonShape shape;
		SetAsBox(shape, 4.0f, 4.0f, Vec2(0.0f, 0.0f), 0_rad);
		m_fixture1 = m_body->CreateFixture(FixtureDef{&shape, 10.0f});

		m_fixture2 = nullptr;

		m_sensor = false;
	}

	void Keyboard(Key key) override
	{
		switch (key)
		{
		case Key_C:
			if (m_fixture2 == nullptr)
			{
				CircleShape shape(3.0, Vec2(0.5f, -4.0f));
				m_fixture2 = m_body->CreateFixture(FixtureDef{&shape, 10.0f});
				m_body->SetAwake();
			}
			break;

		case Key_D:
			if (m_fixture2 != nullptr)
			{
				m_body->DestroyFixture(m_fixture2);
				m_fixture2 = nullptr;
				m_body->SetAwake();
			}
			break;

		case Key_S:
			if (m_fixture2 != nullptr)
			{
				m_sensor = !m_sensor;
				m_fixture2->SetSensor(m_sensor);
			}
			break;

		default:
			break;
		}
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Press: (c) create a shape, (d) destroy a shape.");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "sensor = %d", m_sensor);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new ShapeEditing;
	}

	Body* m_body;
	Fixture* m_fixture1;
	Fixture* m_fixture2;
	bool m_sensor;
};

} // namespace box2d

#endif
