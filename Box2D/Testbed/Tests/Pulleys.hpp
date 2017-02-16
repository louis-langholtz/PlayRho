/*
* Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef PULLEYS_H
#define PULLEYS_H

namespace box2d {

class Pulleys : public Test
{
public:
	Pulleys()
	{
		const auto y = 16.0f;
		const auto L = 12.0f;
		const auto a = 1.0f;
		const auto b = 2.0f;

		const auto ground = m_world->CreateBody();
		{
			CircleShape circle(2.0f, Vec2(-10.0f, y + b + L));
			ground->CreateFixture(std::make_shared<CircleShape>(circle));

			circle.SetLocation(Vec2(10.0f, y + b + L));
			ground->CreateFixture(std::make_shared<CircleShape>(circle));
		}

		{
			const auto shape = std::make_shared<PolygonShape>(a, b);

			BodyDef bd;
			bd.type = BodyType::Dynamic;

			//bd.fixedRotation = true;
			bd.position = Vec2(-10.0f, y);
			const auto body1 = m_world->CreateBody(bd);
			body1->CreateFixture(shape, FixtureDef{}.UseDensity(5));

			bd.position = Vec2(10.0f, y);
			const auto body2 = m_world->CreateBody(bd);
			body2->CreateFixture(shape, FixtureDef{}.UseDensity(5));

			PulleyJointDef pulleyDef;
			Vec2 anchor1(-10.0f, y + b);
			Vec2 anchor2(10.0f, y + b);
			Vec2 groundAnchor1(-10.0f, y + b + L);
			Vec2 groundAnchor2(10.0f, y + b + L);
			pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5f);

			m_joint1 = (PulleyJoint*)m_world->CreateJoint(pulleyDef);
		}
	}

	void PostStep(const Settings&, Drawer& drawer) override
	{
		const auto ratio = m_joint1->GetRatio();
		const auto L = GetCurrentLengthA(*m_joint1) + ratio * GetCurrentLengthB(*m_joint1);
		drawer.DrawString(5, m_textLine, "L1 + %4.2f * L2 = %4.2f", (float) ratio, (float) L);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Pulleys;
	}

	PulleyJoint* m_joint1;
};

} // namespace box2d

#endif
