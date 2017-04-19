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

#ifndef VERTICAL_STACK_H
#define VERTICAL_STACK_H

#include <array>

namespace box2d {

bool g_blockSolve = true;

class VerticalStack : public Test
{
public:

	enum
	{
		e_columnCount = 1,
		e_rowCount = 10
		//e_columnCount = 1,
		//e_rowCount = 1
	};

	VerticalStack()
	{
		m_bulletshape->SetVertexRadius(RealNum{0.25f} * Meter);
		m_bulletshape->SetDensity(RealNum{20} * KilogramPerSquareMeter);
		m_bulletshape->SetRestitution(0.05f);

		const auto ground = m_world->CreateBody();
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(20.0f, 0.0f) * Meter, Vec2(20.0f, 20.0f) * Meter));

		const float xs[] = {0.0f, -10.0f, -5.0f, 5.0f, 10.0f};
		assert(e_columnCount <= sizeof(xs)/sizeof(xs[0]));

		const auto hdim = RealNum{0.1f}; // 0.5f is less stable than 1.0f for boxes not at origin (x of 0)
		const auto shape = std::make_shared<PolygonShape>(hdim * Meter, hdim * Meter);
		shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);
		shape->SetFriction(0.3f);
		for (auto j = 0; j < e_columnCount; ++j)
		{
			for (auto i = 0; i < e_rowCount; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;

				const auto x = 0.0f;
				//const auto x = RandomFloat(-0.02f, 0.02f);
				//const auto x = i % 2 == 0 ? -0.01f : 0.01f;
				//bd.position = Vec2(xs[j] + x, (hdim - hdim/20) + (hdim * 2 - hdim / 20) * i);
				bd.position = Vec2(xs[j] + x, (i + 1) * hdim * 4) * Meter;
				
				const auto body = m_world->CreateBody(bd);
				body->CreateFixture(shape);
			}
		}

		m_bullet = nullptr;
	}

	void KeyboardDown(Key key) override
	{
		switch (key)
		{
		case Key_Comma:
			if (m_bullet)
			{
				m_world->Destroy(m_bullet);
				m_bullet = nullptr;
			}

			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.bullet = true;
				bd.position = Vec2(-31.0f, 5.0f) * Meter;

				m_bullet = m_world->CreateBody(bd);
				m_bullet->CreateFixture(m_bulletshape);
				m_bullet->SetVelocity(Velocity{Vec2(400.0f, 0.0f) * MeterPerSecond, AngularVelocity{0}});
			}
			break;
                
        case Key_B:
            g_blockSolve = !g_blockSolve;
            break;
				
		default:
			break;
		}
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Press: (,) to launch a bullet.");
		m_textLine += DRAW_STRING_NEW_LINE;
		drawer.DrawString(5, m_textLine, "Blocksolve = %d", g_blockSolve);
		//if (GetStepCount() == 300)
		//{
		//	if (m_bullet)
		//	{
		//		m_world->Destroy(m_bullet);
		//		m_bullet = nullptr;
		//	}

		//	{
		//		CircleShape shape;
		//		shape.m_radius = 0.25f;

		//		FixtureDef fd;
		//		fd.shape = &shape;
		//		fd.density = 20.0f;
		//		fd.restitution = 0.05f;

		//		BodyDef bd;
		//		bd.type = BodyType::Dynamic;
		//		bd.bullet = true;
		//		bd.position = Vec2(-31.0f, 5.0f);

		//		m_bullet = m_world->CreateBody(bd);
		//		m_bullet->CreateFixture(fd);

		//		m_bullet->SetLinearVelocity(Vec2(400.0f, 0.0f));
		//	}
		//}
	}

	static Test* Create()
	{
		return new VerticalStack;
	}

	Body* m_bullet;
	std::shared_ptr<CircleShape> m_bulletshape = std::make_shared<CircleShape>();
};
	
} // namespace box2d

#endif
