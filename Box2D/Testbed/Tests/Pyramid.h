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

#ifndef PYRAMID_H
#define PYRAMID_H

namespace box2d {

class Pyramid : public Test
{
public:
	enum
	{
		e_count = 20
	};

	Pyramid()
	{
		{
			BodyDef bd;
			Body* ground = m_world->CreateBody(&bd);

			EdgeShape shape;
			shape.Set(Vec2(-40.0f, 0.0f), Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			float_t a = 0.5f;
			PolygonShape shape;
			shape.SetAsBox(a, a);

			Vec2 x(-7.0f, 0.75f);
			Vec2 y;
			Vec2 deltaX(0.5625f, 1.25f);
			Vec2 deltaY(1.125f, 0.0f);

			for (int32 i = 0; i < e_count; ++i)
			{
				y = x;

				for (int32 j = i; j < e_count; ++j)
				{
					BodyDef bd;
					bd.type = DynamicBody;
					bd.position = y;
					Body* body = m_world->CreateBody(&bd);
					body->CreateFixture(&shape, 5.0f);

					y += deltaY;
				}

				x += deltaX;
			}
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		//DynamicTree* tree = &m_world->m_contactManager.m_broadPhase.m_tree;

		//if (m_stepCount == 400)
		//{
		//	tree->RebuildBottomUp();
		//}
	}

	static Test* Create()
	{
		return new Pyramid;
	}
};

} // namespace box2d

#endif
