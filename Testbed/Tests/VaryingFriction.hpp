/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_VARYING_FRICTION_HPP
#define PLAYRHO_VARYING_FRICTION_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class VaryingFriction : public Test
{
public:

    VaryingFriction()
    {
        m_world.CreateBody()->CreateFixture(Shape(GetGroundEdgeConf()));
        
        const auto sliderPlank = Shape{PolygonShapeConf{}.SetAsBox(13_m, 0.25_m)};
        const auto sliderWall = Shape{PolygonShapeConf{}.SetAsBox(0.25_m, 1_m)};
        
        m_world.CreateBody(BodyDef{}.UseLocation(Vec2(-4, 22) * 1_m).UseAngle(-0.25_rad))->CreateFixture(sliderPlank);
        m_world.CreateBody(BodyDef{}.UseLocation(Vec2(10.5f, 19) * 1_m))->CreateFixture(sliderWall);
        m_world.CreateBody(BodyDef{}.UseLocation(Vec2(4, 14) * 1_m).UseAngle(0.25_rad))->CreateFixture(sliderPlank);
        m_world.CreateBody(BodyDef{}.UseLocation(Vec2(-10.5f, 11) * 1_m))->CreateFixture(sliderWall);
        m_world.CreateBody(BodyDef{}.UseLocation(Vec2(-4, 6) * 1_m).UseAngle(-0.25_rad))->CreateFixture(sliderPlank);
        
        auto shape = PolygonShapeConf{}.SetAsBox(0.5_m, 0.5_m).UseDensity(25_kgpm2);
        float friction[5] = {std::sqrt(std::numeric_limits<float>::max()), 0.5f, 0.35f, 0.1f, 0.0f};
        for (auto i = 0; i < 5; ++i)
        {
            auto bd = BodyDef{};
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-15.0f + 4.0f * i, 28.0f) * 1_m;
            const auto body = m_world.CreateBody(bd);
            shape.UseFriction(Real(friction[i]));
            body->CreateFixture(Shape(shape));
        }
    }
};

} // namespace testbed

#endif
