/*
* Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef AddPair_H
#define AddPair_H

namespace box2d {
	
class AddPair : public Test
{
public:

	AddPair()
	{
		m_world->SetGravity(Vec2(float_t{0}, float_t{0}));
		{
			CircleShape shape{float_t(0.1), Vec2_zero};

			float minX = -6.0f;
			float maxX = 0.0f;
			float minY = 4.0f;
			float maxY = 6.0f;
			
			for (int32 i = 0; i < 400; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(RandomFloat(minX,maxX),RandomFloat(minY,maxY));
				Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(FixtureDef{&shape, 0.01f});
			}
		}
		
		{
			PolygonShape shape;
			shape.SetAsBox(1.5f, 1.5f);
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-40.0f,5.0f);
			bd.bullet = true;
			Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(FixtureDef{&shape, 1.0f});
			body->SetLinearVelocity(Vec2(150.0f, 0.0f));
		}
	}

	static Test* Create()
	{
		return new AddPair;
	}
};

} // namespace box2d

#endif
