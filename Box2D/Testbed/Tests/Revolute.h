/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
		Body* ground = nullptr;
		{
			BodyDef bd;
			ground = m_world->CreateBody(&bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));

			FixtureDef fd;
			fd.shape = &shape;
			//fd.filter.categoryBits = 2;

			ground->CreateFixture(&fd);
		}

		{
			b2CircleShape shape;
			shape.SetRadius(0.5);

			BodyDef bd;
			bd.type = DynamicBody;

			RevoluteJointDef rjd;

			bd.position = Vec2(-10.0f, 20.0f);
			Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 5.0f);

			float_t w = 100.0f;
			body->SetAngularVelocity(w);
			body->SetLinearVelocity(Vec2(-8.0f * w, 0.0f));

			rjd.Initialize(ground, body, Vec2(-10.0f, 12.0f));
			rjd.motorSpeed = 1.0f * Pi;
			rjd.maxMotorTorque = 10000.0f;
			rjd.enableMotor = false;
			rjd.lowerAngle = -0.25f * Pi;
			rjd.upperAngle = 0.5f * Pi;
			rjd.enableLimit = true;
			rjd.collideConnected = true;

			m_joint = (RevoluteJoint*)m_world->CreateJoint(&rjd);
		}

		{
			b2CircleShape circle_shape;
			circle_shape.SetRadius(3.0);

			BodyDef circle_bd;
			circle_bd.type = DynamicBody;
			circle_bd.position = Vec2(5.0f, 30.0f);

			FixtureDef fd;
			fd.density = 5.0f;
			fd.filter.maskBits = 1;
			fd.shape = &circle_shape;

			m_ball = m_world->CreateBody(&circle_bd);
			m_ball->CreateFixture(&fd);

			b2PolygonShape polygon_shape;
			polygon_shape.SetAsBox(10.0f, 0.2f, Vec2 (-10.0f, 0.0f), 0.0f);

			BodyDef polygon_bd;
			polygon_bd.position = Vec2(20.0f, 10.0f);
			polygon_bd.type = DynamicBody;
			polygon_bd.bullet = true;
			Body* polygon_body = m_world->CreateBody(&polygon_bd);
			polygon_body->CreateFixture(&polygon_shape, 2.0f);

			RevoluteJointDef rjd;
			rjd.Initialize(ground, polygon_body, Vec2(20.0f, 10.0f));
			rjd.lowerAngle = -0.25f * Pi;
			rjd.upperAngle = 0.0f * Pi;
			rjd.enableLimit = true;
			m_world->CreateJoint(&rjd);
		}

		// Tests mass computation of a small object far from the origin
		{
			BodyDef bodyDef;
			bodyDef.type = DynamicBody;
			Body* body = m_world->CreateBody(&bodyDef);
		
			b2PolygonShape polyShape;		
			Vec2 verts[3];
			verts[0] = Vec2( 17.63f, 36.31f );
			verts[1] = Vec2( 17.52f, 36.69f );
			verts[2] = Vec2( 17.19f, 36.36f );
			polyShape.Set(verts, 3);
		
			FixtureDef polyFixtureDef;
			polyFixtureDef.shape = &polyShape;
			polyFixtureDef.density = 1;

			body->CreateFixture(&polyFixtureDef);	//assertion hits inside here
		}

	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_L:
			m_joint->EnableLimit(!m_joint->IsLimitEnabled());
			break;

		case GLFW_KEY_M:
			m_joint->EnableMotor(!m_joint->IsMotorEnabled());
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, m_textLine, "Keys: (l) limits, (m) motor");
		m_textLine += DRAW_STRING_NEW_LINE;

		//if (m_stepCount == 360)
		//{
		//	m_ball->SetTransform(Vec2(0.0f, 0.5f), 0.0f);
		//}

		//float_t torque1 = m_joint1->GetMotorTorque();
		//g_debugDraw.DrawString(5, m_textLine, "Motor Torque = %4.0f, %4.0f : Motor Force = %4.0f", (float) torque1, (float) torque2, (float) force3);
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
