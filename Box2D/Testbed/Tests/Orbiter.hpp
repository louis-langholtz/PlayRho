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

#ifndef Orbiter_hpp
#define Orbiter_hpp

namespace box2d {
	
	class Orbiter: public Test
	{
	public:
		
		Orbiter()
		{
			m_world->SetGravity(Vec2{0, 0});

			BodyDef bd;
			const auto radius = 12.0f;

			bd.type = BodyType::Static;
			bd.position = m_center;
			const auto ctrBody = m_world->CreateBody(bd);
			const auto ctrShape = std::make_shared<CircleShape>();
			ctrShape->SetRadius(3);
			ctrBody->CreateFixture(ctrShape);

			bd.type = BodyType::Dynamic;
			bd.position = Vec2{m_center.x, m_center.y + radius};
			m_orbiter = m_world->CreateBody(bd);
			const auto ballShape = std::make_shared<CircleShape>();
			ballShape->SetRadius(0.5f);
			m_orbiter->CreateFixture(ballShape);
			
			const auto velocity = Velocity{
				Vec2{Pi * radius / 2, 0},
				360.0f * Degree
			};
			m_orbiter->SetVelocity(velocity);
		}
		
		void PreStep(const Settings&, Drawer&) override
		{
			const auto force = GetCentripetalForce(*m_orbiter, m_center);
			const auto linAccel = force * RealNum{m_orbiter->GetInvMass() * Kilogram};
			const auto angAccel = 0.0f * Degree;
			m_orbiter->SetAcceleration(linAccel, angAccel);
		}
		
		static Test* Create()
		{
			return new Orbiter;
		}
		
	private:
		Body* m_orbiter = nullptr;
		Vec2 const m_center = {0, 20};

	};
	
}

#endif /* Orbiter_hpp */
