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

#ifndef MOBILE_H
#define MOBILE_H

namespace box2d {

class Mobile : public Test
{
public:

	enum
	{
		e_depth = 4
	};

	Mobile()
	{
		Body* ground;

		// Create ground body.
		{
			BodyDef bodyDef;
			bodyDef.position = Vec2(0.0f, 20.0f);
			ground = m_world->CreateBody(&bodyDef);
		}

		float a = 0.5f;
		Vec2 h(0.0f, a);

		Body* root = AddNode(ground, Vec2_zero, 0, 3.0f, a);

		RevoluteJointDef jointDef;
		jointDef.bodyA = ground;
		jointDef.bodyB = root;
		jointDef.localAnchorA = Vec2_zero;
		jointDef.localAnchorB = h;
		m_world->CreateJoint(&jointDef);
	}

	Body* AddNode(Body* parent, const Vec2& localAnchor, int32 depth, float offset, float a)
	{
		float density = 20.0f;
		Vec2 h(0.0f, a);

		Vec2 p = parent->GetPosition() + localAnchor - h;

		BodyDef bodyDef;
		bodyDef.type = BodyType::Dynamic;
		bodyDef.position = p;
		Body* body = m_world->CreateBody(&bodyDef);

		PolygonShape shape;
		shape.SetAsBox(0.25f * a, a);
		body->CreateFixture(FixtureDef{&shape, density});

		if (depth == e_depth)
		{
			return body;
		}

		Vec2 a1 = Vec2(offset, -a);
		Vec2 a2 = Vec2(-offset, -a);
		Body* body1 = AddNode(body, a1, depth + 1, 0.5f * offset, a);
		Body* body2 = AddNode(body, a2, depth + 1, 0.5f * offset, a);

		RevoluteJointDef jointDef;
		jointDef.bodyA = body;
		jointDef.localAnchorB = h;

		jointDef.localAnchorA = a1;
		jointDef.bodyB = body1;
		m_world->CreateJoint(&jointDef);

		jointDef.localAnchorA = a2;
		jointDef.bodyB = body2;
		m_world->CreateJoint(&jointDef);

		return body;
	}

	static Test* Create()
	{
		return new Mobile;
	}
};

} // namespace box2d

#endif
