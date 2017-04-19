/*
* Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef TUMBLER_H
#define TUMBLER_H

namespace box2d {

class Tumbler : public Test
{
public:

	enum
	{
		e_count = 800
	};

	Tumbler()
	{
		m_shape->SetDensity(RealNum{1} * KilogramPerSquareMeter);

		const auto g = m_world->CreateBody(BodyDef{}.UseType(BodyType::Static));

		const auto b = m_world->CreateBody(BodyDef{}.UseType(BodyType::Dynamic)
										   .UseLocation(Vec2(0, 10) * Meter)
										   .UseAllowSleep(false));

		PolygonShape shape;
		shape.SetDensity(RealNum{5} * KilogramPerSquareMeter);
		SetAsBox(shape, RealNum{0.5f} * Meter, RealNum{10.0f} * Meter, Vec2( 10.0f, 0.0f) * Meter, Angle{0});
		b->CreateFixture(std::make_shared<PolygonShape>(shape));
		SetAsBox(shape, RealNum{0.5f} * Meter, RealNum{10.0f} * Meter, Vec2(-10.0f, 0.0f) * Meter, Angle{0});
		b->CreateFixture(std::make_shared<PolygonShape>(shape));
		SetAsBox(shape, RealNum{10.0f} * Meter, RealNum{0.5f} * Meter, Vec2(0.0f, 10.0f) * Meter, Angle{0});
		b->CreateFixture(std::make_shared<PolygonShape>(shape));
		SetAsBox(shape, RealNum{10.0f} * Meter, RealNum{0.5f} * Meter, Vec2(0.0f, -10.0f) * Meter, Angle{0});
		b->CreateFixture(std::make_shared<PolygonShape>(shape));

		RevoluteJointDef jd;
		jd.bodyA = g;
		jd.bodyB = b;
		jd.localAnchorA = Vec2(0.0f, 10.0f) * Meter;
		jd.localAnchorB = Vec2(0.0f, 0.0f) * Meter;
		jd.referenceAngle = Angle{0};
		jd.motorSpeed = 0.05f * Pi * RadianPerSecond;
		jd.maxMotorTorque = RealNum{100000} * NewtonMeter; // 1e8f;
		jd.enableMotor = true;
		m_joint = static_cast<RevoluteJoint*>(m_world->CreateJoint(jd));
	}

	void PostStep(const Settings&, Drawer&) override
	{
		if (m_count < e_count)
		{
			const auto body = m_world->CreateBody(BodyDef{}
												  .UseType(BodyType::Dynamic)
												  .UseLocation(Vec2(0, 10) * Meter));
			body->CreateFixture(m_shape);
			++m_count;
		}
	}

	void KeyboardDown(Key key) override
	{
		switch (key)
		{
			case Key_Add:
				m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() + 0.01f * Pi * RadianPerSecond);
				break;
			
			case Key_Subtract:
				m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() - 0.01f * Pi * RadianPerSecond);
				break;
	
			default:
				break;
		}
	}
	
	static Test* Create()
	{
		return new Tumbler;
	}

	RevoluteJoint* m_joint;
	int32 m_count = 0;
	std::shared_ptr<PolygonShape> m_shape = std::make_shared<PolygonShape>(RealNum{0.125f} * Meter, RealNum{0.125f} * Meter);
};

} // namespace box2d

#endif
