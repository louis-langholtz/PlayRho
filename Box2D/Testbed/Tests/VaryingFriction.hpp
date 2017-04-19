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

#ifndef VARYING_FRICTION_H
#define VARYING_FRICTION_H

namespace box2d {

class VaryingFriction : public Test
{
public:

	VaryingFriction()
	{
		{
			const auto ground = m_world->CreateBody();
			ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));
		}

		const auto sliderPlank = std::make_shared<PolygonShape>(RealNum{13.0f} * Meter, RealNum{0.25f} * Meter);
		const auto sliderWall = std::make_shared<PolygonShape>(RealNum{0.25f} * Meter, RealNum{1.0f} * Meter);
		
		{
			BodyDef bd;
			bd.position = Vec2(-4.0f, 22.0f) * Meter;
			bd.angle = RealNum{-0.25f} * Radian;

			const auto ground = m_world->CreateBody(bd);
			ground->CreateFixture(sliderPlank);
		}

		{
			BodyDef bd;
			bd.position = Vec2(10.5f, 19.0f) * Meter;

			const auto ground = m_world->CreateBody(bd);
			ground->CreateFixture(sliderWall);
		}

		{
			BodyDef bd;
			bd.position = Vec2(4.0f, 14.0f) * Meter;
			bd.angle = RealNum{0.25f} * Radian;

			const auto ground = m_world->CreateBody(bd);
			ground->CreateFixture(sliderPlank);
		}

		{
			BodyDef bd;
			bd.position = Vec2(-10.5f, 11.0f) * Meter;

			const auto ground = m_world->CreateBody(bd);
			ground->CreateFixture(sliderWall);
		}

		{
			BodyDef bd;
			bd.position = Vec2(-4.0f, 6.0f) * Meter;
			bd.angle = RealNum{-0.25f} * Radian;

			const auto ground = m_world->CreateBody(bd);
			ground->CreateFixture(sliderPlank);
		}

		{
			auto shape = PolygonShape(RealNum{0.5f} * Meter, RealNum{0.5f} * Meter);
			shape.SetDensity(RealNum{25} * KilogramPerSquareMeter);

			float friction[5] = {std::numeric_limits<float>::infinity(), 0.5f, 0.35f, -0.1f, 0.0f};
			for (auto i = 0; i < 5; ++i)
			{
				BodyDef bd;
				bd.type = BodyType::Dynamic;
				bd.position = Vec2(-15.0f + 4.0f * i, 28.0f) * Meter;
				const auto body = m_world->CreateBody(bd);

				shape.SetFriction(friction[i]);
				body->CreateFixture(std::make_shared<PolygonShape>(shape));
			}
		}
	}

	static Test* Create()
	{
		return new VaryingFriction;
	}
};

} // namespace box2d

#endif
