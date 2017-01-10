/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef MOBILE_BALANCED_H
#define MOBILE_BALANCED_H

namespace box2d {

class MobileBalanced : public Test
{
public:

	enum
	{
		e_depth = 4
	};

	MobileBalanced()
	{
		const auto ground = m_world->CreateBody(BodyDef{}.UseLocation(Vec2(0.0f, 20.0f)));

		const auto a = 0.5f;
		Vec2 h(0.0f, a);

		const auto root = AddNode(ground, Vec2_zero, 0, 3.0f, a);

		RevoluteJointDef jointDef;
		jointDef.bodyA = ground;
		jointDef.bodyB = root;
		jointDef.localAnchorA = Vec2_zero;
		jointDef.localAnchorB = h;
		m_world->CreateJoint(jointDef);
	}

	Body* AddNode(Body* parent, const Vec2& localAnchor, int32 depth, float offset, float a)
	{
		const auto density = 20.0f;
		const auto h = Vec2(0.0f, a);

		const auto p = parent->GetLocation() + localAnchor - h;

		BodyDef bodyDef;
		bodyDef.type = BodyType::Dynamic;
		bodyDef.position = p;
		const auto body = m_world->CreateBody(bodyDef);

		PolygonShape shape;
		shape.SetAsBox(0.25f * a, a);
		body->CreateFixture(std::make_shared<PolygonShape>(shape), FixtureDef{}.UseDensity(density));

		if (depth == e_depth)
		{
			return body;
		}

		SetAsBox(shape, offset, 0.25f * a, Vec2(0, -a), 0.0_rad);
		body->CreateFixture(std::make_shared<PolygonShape>(shape), FixtureDef{}.UseDensity(density));

		const auto a1 = Vec2(offset, -a);
		const auto a2 = Vec2(-offset, -a);
		const auto body1 = AddNode(body, a1, depth + 1, 0.5f * offset, a);
		const auto body2 = AddNode(body, a2, depth + 1, 0.5f * offset, a);

		RevoluteJointDef jointDef;
		jointDef.bodyA = body;
		jointDef.localAnchorB = h;

		jointDef.localAnchorA = a1;
		jointDef.bodyB = body1;
		m_world->CreateJoint(jointDef);

		jointDef.localAnchorA = a2;
		jointDef.bodyB = body2;
		m_world->CreateJoint(jointDef);

		return body;
	}

	static Test* Create()
	{
		return new MobileBalanced;
	}
};

} // namespace box2d

#endif
