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

#ifndef VARYING_RESTITUTION_H
#define VARYING_RESTITUTION_H

namespace box2d {

// Note: even with a restitution of 1.0, there is some energy change
// due to position correction.
class VaryingRestitution : public Test
{
public:

	VaryingRestitution()
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
			shape.SetRadius(float_t(1));

			FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;

			float_t restitution[7] = {0.0f, 0.1f, 0.3f, 0.5f, 0.75f, 0.9f, 1.0f};

			for (int32 i = 0; i < 7; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(-10.0f + 3.0f * i, 20.0f);

				Body* body = m_world->CreateBody(bd);

				fd.restitution = restitution[i];
				body->CreateFixture(fd);
			}
		}
	}

	static Test* Create()
	{
		return new VaryingRestitution;
	}
};

} // namespace box2d

#endif
