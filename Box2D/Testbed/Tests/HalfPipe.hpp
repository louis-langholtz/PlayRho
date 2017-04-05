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

#ifndef HalfPipe_hpp
#define HalfPipe_hpp

namespace box2d {
	
	class HalfPipe : public Test
	{
	public:
		
		HalfPipe()
		{
			const auto pipeBody = m_world->CreateBody();
			auto pipeShape = std::make_shared<ChainShape>();
			pipeShape->SetFriction(1.0f);
			{
				auto vertices = std::vector<Vec2>();
				const auto pipeRadius = 20.0f;
				for (auto i = 0; i < 90; ++i)
				{
					const auto angle = RealNum{((i * 2 + 180.0f) * Degree) / Radian};
					const auto x = pipeRadius * std::cos(angle);
					const auto y = pipeRadius * std::sin(angle);
					vertices.push_back(Vec2{x, y + 20});
				}
				pipeShape->CreateChain(Span<const Vec2>(vertices.data(), vertices.size()));
			}
			pipeBody->CreateFixture(pipeShape);
			
			BodyDef bd;
			bd.type = BodyType::Dynamic;
			bd.position = Vec2(-19, 28);
			const auto ballBody = m_world->CreateBody(bd);
			auto conf = CircleShape::Conf{};
			conf.density = RealNum{0.01f} * KilogramPerSquareMeter;
			conf.vertexRadius = 1;
			conf.friction = 1.0f;
			ballBody->CreateFixture(std::make_shared<CircleShape>(conf));
		}
		
		static Test* Create()
		{
			return new HalfPipe;
		}
	};
	
}

#endif /* HalfPipe_hpp */
