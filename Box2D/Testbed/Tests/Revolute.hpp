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

#ifndef REVOLUTE_H
#define REVOLUTE_H

namespace box2d {

class Revolute : public Test
{
public:
	Revolute()
	{
		const auto ground = m_world->CreateBody();
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f)));

		{
			BodyDef bd;
			bd.type = BodyType::Dynamic;

			bd.position = Vec2(-10.0f, 20.0f);
			const auto body = m_world->CreateBody(bd);
			auto circleConf = CircleShape::Conf{};
			circleConf.vertexRadius = 0.5f;
			circleConf.density = RealNum{5} * KilogramPerSquareMeter;
			body->CreateFixture(std::make_shared<CircleShape>(circleConf));

			const auto w = 100.0f;
			body->SetVelocity(Velocity{Vec2(-8.0f * w, 0.0f), 1_rad * w});
			
			RevoluteJointDef rjd(ground, body, Vec2(-10.0f, 12.0f));
			rjd.motorSpeed = 1.0f * Pi;
			rjd.maxMotorTorque = 10000.0f;
			rjd.enableMotor = false;
			rjd.lowerAngle = -0.25_rad * Pi;
			rjd.upperAngle = 0.5_rad * Pi;
			rjd.enableLimit = true;
			rjd.collideConnected = true;

			m_joint = (RevoluteJoint*)m_world->CreateJoint(rjd);
		}

		{
			BodyDef circle_bd;
			circle_bd.type = BodyType::Dynamic;
			circle_bd.position = Vec2(5.0f, 30.0f);

			FixtureDef fd;
			fd.filter.maskBits = 1;

			m_ball = m_world->CreateBody(circle_bd);
			auto circleConf = CircleShape::Conf{};
			circleConf.vertexRadius = 3.0f;
			circleConf.density = RealNum{5} * KilogramPerSquareMeter;
			m_ball->CreateFixture(std::make_shared<CircleShape>(circleConf), fd);

			PolygonShape polygon_shape;
			SetAsBox(polygon_shape, 10.0f, 0.2f, Vec2 (-10.0f, 0.0f), 0_rad);
			polygon_shape.SetDensity(RealNum{2} * KilogramPerSquareMeter);

			BodyDef polygon_bd;
			polygon_bd.position = Vec2(20.0f, 10.0f);
			polygon_bd.type = BodyType::Dynamic;
			polygon_bd.bullet = true;
			const auto polygon_body = m_world->CreateBody(polygon_bd);
			polygon_body->CreateFixture(std::make_shared<PolygonShape>(polygon_shape));

			RevoluteJointDef rjd(ground, polygon_body, Vec2(20.0f, 10.0f));
			rjd.lowerAngle = -0.25_rad * Pi;
			rjd.upperAngle = 0.0_rad * Pi;
			rjd.enableLimit = true;
			m_world->CreateJoint(rjd);
		}

		// Tests mass computation of a small object far from the origin
		{
			BodyDef bodyDef;
			bodyDef.type = BodyType::Dynamic;
			const auto body = m_world->CreateBody(bodyDef);
		
			auto polyShape = PolygonShape({Vec2(17.63f, 36.31f), Vec2(17.52f, 36.69f), Vec2(17.19f, 36.36f)});
			polyShape.SetDensity(RealNum{1} * KilogramPerSquareMeter);
		
			body->CreateFixture(std::make_shared<PolygonShape>(polyShape));	//assertion hits inside here
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
				
		default:
			break;
		}
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		drawer.DrawString(5, m_textLine, "Keys: (l) limits, (m) motor");
		m_textLine += DRAW_STRING_NEW_LINE;

		//if (GetStepCount() == 360)
		//{
		//	m_ball->SetTransform(Vec2(0.0f, 0.5f), 0.0f);
		//}

		//RealNum torque1 = m_joint1->GetMotorTorque();
		//drawer.DrawString(5, m_textLine, "Motor Torque = %4.0f, %4.0f : Motor Force = %4.0f", (float) torque1, (float) torque2, (float) force3);
		//m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Revolute;
	}

	Body* m_ball;
	RevoluteJoint* m_joint;
};

} // namespace box2d

#endif
