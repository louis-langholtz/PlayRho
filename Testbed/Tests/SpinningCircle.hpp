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

#ifndef PLAYRHO_SPINNING_CIRCLE_HPP
#define PLAYRHO_SPINNING_CIRCLE_HPP

#include "../Framework/Test.hpp"

namespace testbed {
    
    class SpinningCircle : public Test
    {
    public:
        
        enum
        {
            e_count = 10
        };
        
        SpinningCircle()
        {
            m_world.SetGravity(LinearAcceleration2{});

            auto bodyDef = BodyDef{};
            bodyDef.type = BodyType::Dynamic;
            bodyDef.angularVelocity = 45_deg / 1_s;
            bodyDef.linearVelocity = LinearVelocity2{};
            bodyDef.linearDamping = 0.8_Hz;
            bodyDef.bullet = true;

            bodyDef.location = Vec2{0, 26} * 1_m;
            const auto body1 = m_world.CreateBody(bodyDef);
            bodyDef.location = Vec2{0, 14} * 1_m;
            const auto body2 = m_world.CreateBody(bodyDef);
            
            auto shapeConf = DiskShapeConf{};
            shapeConf.density = 10_kgpm2;

            shapeConf.vertexRadius = 2_m;
            shapeConf.location = Length2{};
            auto circle = Shape(shapeConf);

            shapeConf.vertexRadius = 1.5_m;
            shapeConf.location = Vec2{0,  3} * 1_m;
            auto circleA = Shape(shapeConf);
            shapeConf.vertexRadius = 1.5_m;
            shapeConf.location = Vec2{0, -3} * 1_m;
            auto circleB = Shape(shapeConf);
            
            body1->CreateFixture(circleA);
            body1->CreateFixture(circleB);
            
            body2->CreateFixture(circleA);
            body2->CreateFixture(circleB);
        }
    };
    
} // namespace testbed

#endif
