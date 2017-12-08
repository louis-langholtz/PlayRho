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

#ifndef PLAYRHO_CONTINUOUS_TEST_HPP
#define  PLAYRHO_CONTINUOUS_TEST_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class ContinuousTest : public Test
{
public:

    ContinuousTest()
    {
        {
            BodyDef bd;
            bd.location = Length2{};
            Body* body = m_world.CreateBody(bd);

            body->CreateFixture(std::make_shared<EdgeShape>(Vec2(-10.0f, 0.0f) * 1_m, Vec2(10.0f, 0.0f) * 1_m));

            const auto shape = PolygonShape::Conf{}.SetAsBox(0.2_m, 1_m, Vec2(0.5f, 1.0f) * 1_m, 0_rad);
            body->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(0.0f, 20.0f) * 1_m;
            //bd.angle = 0.1f;

            const auto shape = std::make_shared<PolygonShape>(2_m, 0.1_m,
                                                              PolygonShape::Conf{}.SetDensity(1_kgpm2));
            m_body = m_world.CreateBody(bd);
            m_body->CreateFixture(shape);
            m_angularVelocity = RandomFloat(-50.0f, 50.0f) * 1_rad / 1_s;
            //m_angularVelocity = 46.661274f;
            m_body->SetVelocity(Velocity{Vec2(0.0f, -100.0f) * 1_mps, m_angularVelocity});
        }
    }

    void Launch()
    {
        m_body->SetTransform(Vec2(0.0f, 20.0f) * 1_m, 0_rad);
        m_angularVelocity = RandomFloat(-50.0f, 50.0f) * 1_rad / 1_s;
        m_body->SetVelocity(Velocity{Vec2(0.0f, -100.0f) * 1_mps, m_angularVelocity});
    }

    void PostStep(const Settings&, Drawer&) override
    {
        if (GetStepCount() % 60 == 0)
        {
            Launch();
        }
    }

    Body* m_body;
    AngularVelocity m_angularVelocity;
};

} // namespace playrho

#endif
