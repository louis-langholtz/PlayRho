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

#ifndef PRISMATIC_H
#define PRISMATIC_H

namespace box2d {

// The motor in this test gets smoother with higher velocity iterations.
class Prismatic : public Test
{
public:
	Prismatic()
	{
		const auto ground = m_world->CreateBody();
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f)));

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-10.0f, 10.0f);
			bd.angle = 0.5f * Radian * Pi;
			bd.allowSleep = false;
			const auto body = m_world->CreateBody(bd);
			
			auto polygonConf = PolygonShape::Conf{};
			polygonConf.density = RealNum{5} * KilogramPerSquareMeter;
			body->CreateFixture(std::make_shared<PolygonShape>(2.0f, 0.5f, polygonConf));

			// Bouncy limit
			const auto axis = GetUnitVector(Vec2(2.0f, 1.0f));
			PrismaticJointDef pjd(ground, body, Vec2(0.0f, 0.0f), GetVec2(axis));

			// Non-bouncy limit
			//pjd.Initialize(ground, body, Vec2(-10.0f, 10.0f), Vec2(1.0f, 0.0f));

			pjd.motorSpeed = 10.0f;
			pjd.maxMotorForce = 10000.0f;
			pjd.enableMotor = true;
			pjd.lowerTranslation = 0.0f;
			pjd.upperTranslation = 20.0f;
			pjd.enableLimit = true;

			m_joint = (PrismaticJoint*)m_world->CreateJoint(pjd);
		}
	}

	void KeyboardDown(Key key) override
	{
		switch (key)
		{
		case Key_L:
			m_joint->EnableLimit(!m_joint->IsLimitEnabled());
			break;

		case Key_M:
			m_joint->EnableMotor(!m_joint->IsMotorEnabled());
			break;

		case Key_S:
			m_joint->SetMotorSpeed(-m_joint->GetMotorSpeed());
			break;

		default:
			break;
		}
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Keys: (l) limits, (m) motors, (s) speed");
		m_textLine += DRAW_STRING_NEW_LINE;
		const auto force = m_joint->GetMotorForce(settings.hz);
		drawer.DrawString(5, m_textLine, "Motor Force = %4.0f", (float) force);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Prismatic;
	}

	PrismaticJoint* m_joint;
};

} // namespace box2d

#endif
