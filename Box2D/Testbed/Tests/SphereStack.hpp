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

#ifndef SPHERE_STACK_H
#define SPHERE_STACK_H

namespace box2d {

class SphereStack : public Test
{
public:

	enum
	{
		e_count = 10
	};

	SphereStack()
	{
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(FixtureDef{&shape, 0.0f});
		}

		{
			CircleShape shape;
			shape.SetRadius(1.0);

			for (int32 i = 0; i < e_count; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(0.0, 4.0f + 3.0f * i);

				m_bodies[i] = m_world->CreateBody(bd);

				m_bodies[i]->CreateFixture(FixtureDef{&shape, 1.0f});

				m_bodies[i]->SetVelocity(Velocity{Vec2(0.0f, -50.0f), 0_rad});
			}
		}
	}

	void PostStep(const Settings& settings, Drawer& drawer) override
	{
		//for (int32 i = 0; i < e_count; ++i)
		//{
		//	printf("%g ", m_bodies[i]->GetWorldCenter().y);
		//}

		//for (int32 i = 0; i < e_count; ++i)
		//{
		//	printf("%g ", m_bodies[i]->GetLinearVelocity().y);
		//}

		//printf("\n");
	}

	static Test* Create()
	{
		return new SphereStack;
	}

	Body* m_bodies[e_count];
};

} // namespace box2d

#endif
