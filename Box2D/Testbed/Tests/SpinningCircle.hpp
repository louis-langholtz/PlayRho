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
			m_world->SetGravity(Vec2{0, 0} * MeterPerSquareSecond);

			auto bodyDef = BodyDef{};
			bodyDef.type = BodyType::Dynamic;
			bodyDef.angularVelocity = 45.0f * Degree / Second;
			bodyDef.linearVelocity = Vec2{0, 0} * MeterPerSecond;
			bodyDef.linearDamping = 0.8f;
			bodyDef.bullet = true;

			bodyDef.position = Vec2{0, 26};
			const auto body1 = m_world->CreateBody(bodyDef);
			bodyDef.position = Vec2{0, 14};
			const auto body2 = m_world->CreateBody(bodyDef);
			
			auto shapeConf = CircleShape::Conf{};
			shapeConf.density = RealNum{10} * KilogramPerSquareMeter;

			shapeConf.vertexRadius = 2;
			shapeConf.location = Vec2{0, 0};
			auto circle = std::make_shared<CircleShape>(shapeConf);

			shapeConf.vertexRadius = 1.5;
			shapeConf.location = Vec2{0,  3};
			auto circleA = std::make_shared<CircleShape>(shapeConf);
			shapeConf.vertexRadius = 1.5;
			shapeConf.location = Vec2{0, -3};
			auto circleB = std::make_shared<CircleShape>(shapeConf);
			
			body1->CreateFixture(circleA);
			body1->CreateFixture(circleB);
			
			body2->CreateFixture(circleA);
			body2->CreateFixture(circleB);
		}
		
		static Test* Create()
		{
			return new SpinningCircle;
		}
	};
	
} // namespace box2d

#endif
