/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

namespace playrho {
    
    class HalfPipe : public Test
    {
    public:
        
        HalfPipe()
        {
            const auto pipeBody = m_world->CreateBody(BodyDef{}.UseLocation(Vec2(0, 20) * Meter));
            {
                auto conf = ChainShape::Conf{};
                conf.UseFriction(1.0f);
                conf.vertices = GetCircleVertices(Real{20.0f} * Meter, 90,
                                                  Real(180) * Degree, Real(0.5f));
                pipeBody->CreateFixture(std::make_shared<ChainShape>(conf));
            }

            const auto ballBody = m_world->CreateBody(BodyDef{}
                                                      .UseType(BodyType::Dynamic)
                                                      .UseLocation(Vec2(-19, 28) * Meter));
            ballBody->CreateFixture(std::make_shared<DiskShape>(DiskShape::Conf{}
                                    .UseDensity(Real{0.01f} * KilogramPerSquareMeter)
                                    .UseVertexRadius(Real{1} * Meter)
                                    .UseFriction(1.0f)));
        }
    };
    
}

#endif /* HalfPipe_hpp */
