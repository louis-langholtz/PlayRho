/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef SPINNING_CIRCLE_HPP
#define SPINNING_CIRCLE_HPP

namespace box2d {
	
	class SpinningCircle : public Test
	{
	public:
		
		enum
		{
			e_count = 10
		};
		
		SpinningCircle()
		{
			m_world->SetGravity(Vec2{0, 0});

			auto bodyDef = BodyDef{};
			bodyDef.type = BodyType::Dynamic;
			bodyDef.angularVelocity = 45_deg;
			bodyDef.linearVelocity = Vec2{0, 0};
			bodyDef.linearDamping = 0.8f;

			bodyDef.position = Vec2{ 6, 20};
			const auto body1 = m_world->CreateBody(bodyDef);
			bodyDef.position = Vec2{-6, 20};
			const auto body2 = m_world->CreateBody(bodyDef);
			
			auto shapeConf = CircleShape::Conf{};
			shapeConf.vertexRadius = 2;
			shapeConf.density = 1;
			shapeConf.location = Vec2{0, 0};
			auto circle = std::make_shared<CircleShape>(shapeConf);
			
			auto fixtureConf = FixtureDef{};
			
			fixtureConf.location = Vec2{0, 3};
			body1->CreateFixture(circle, fixtureConf);
			fixtureConf.location = Vec2{0, -3};
			body1->CreateFixture(circle, fixtureConf);

			fixtureConf.location = Vec2{0,  3};
			body2->CreateFixture(circle, fixtureConf);
			fixtureConf.location = Vec2{0, -3};
			body2->CreateFixture(circle, fixtureConf);
		}
		
		static Test* Create()
		{
			return new SpinningCircle;
		}
	};
	
} // namespace box2d

#endif
