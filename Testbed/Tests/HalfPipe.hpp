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

#include "../Framework/Test.hpp"

namespace box2d {
    
    class HalfPipe : public Test
    {
    public:
        
        HalfPipe()
        {
            const auto pipeBody = m_world->CreateBody();

            {
                auto conf = ChainShape::Conf{};
                conf.UseFriction(1.0f);
                const auto pipeRadius = RealNum{20.0f} * Meter;
                for (auto i = 0; i < 90; ++i)
                {
                    const auto angle = RealNum{(RealNum(i * 2 + 180.0f) * Degree) / Radian};
                    const auto x = pipeRadius * RealNum{std::cos(angle)};
                    const auto y = pipeRadius * RealNum{std::sin(angle)};
                    conf.vertices.push_back(Length2D{x, y + RealNum{20} * Meter});
                }
                pipeBody->CreateFixture(std::make_shared<ChainShape>(conf));
            }
            
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(-19, 28) * Meter;
            const auto ballBody = m_world->CreateBody(bd);
            auto conf = DiskShape::Conf{};
            conf.density = RealNum{0.01f} * KilogramPerSquareMeter;
            conf.vertexRadius = RealNum{1} * Meter;
            conf.friction = 1.0f;
            ballBody->CreateFixture(std::make_shared<DiskShape>(conf));
        }
    };
    
}

#endif /* HalfPipe_hpp */
