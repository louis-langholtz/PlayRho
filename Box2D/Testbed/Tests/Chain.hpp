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

#ifndef CHAIN_H
#define CHAIN_H

namespace box2d {

class Chain : public Test
{
public:
	Chain()
	{
		const auto ground = m_world->CreateBody();
		ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));
	
		{
			const auto shape = std::make_shared<PolygonShape>(0.6f * Meter, 0.125f * Meter);
			shape->SetDensity(RealNum{20} * KilogramPerSquareMeter);
			shape->SetFriction(0.2f);

			const auto y = 25.0f;
			auto prevBody = ground;
			for (auto i = 0; i < 30; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(0.5f + i, y) * Meter;
				const auto body = m_world->CreateBody(bd);
				body->CreateFixture(shape);

				m_world->CreateJoint(RevoluteJointDef(prevBody, body, Vec2(RealNum(i), y) * Meter));

				prevBody = body;
			}
		}
	}

	static Test* Create()
	{
		return new Chain;
	}
};

} // namespace box2d

#endif
