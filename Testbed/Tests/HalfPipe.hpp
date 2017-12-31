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

#ifndef PLAYRHO_HALF_PIPE_HPP
#define PLAYRHO_HALF_PIPE_HPP

#include "../Framework/Test.hpp"

namespace testbed {
    
    class HalfPipe : public Test
    {
    public:
        HalfPipe()
        {
            const auto pipeBody = m_world.CreateBody(BodyConf{}.UseLocation(Vec2(0, 20) * 1_m));
            {
                auto conf = ChainShapeConf{};
                conf.UseFriction(Real(1));
                conf.Set(GetCircleVertices(20_m, 90, 180_deg, Real(0.5f)));
                pipeBody->CreateFixture(Shape{conf});
            }
            const auto ballBody = m_world.CreateBody(BodyConf{}
                                                      .UseType(BodyType::Dynamic)
                                                      .UseLocation(Vec2(-19, 28) * 1_m));
            ballBody->CreateFixture(DiskShapeConf{}.UseDensity(0.01_kgpm2).UseRadius(1_m).UseFriction(Real(1)));
        }
    };
    
}

#endif /* PLAYRHO_HALF_PIPE_HPP */
