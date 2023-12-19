/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "../Framework/Test.hpp"

namespace testbed {

class BulletTest : public Test
{
public:
    static inline const auto registered = RegisterTest("Bullet Test", MakeUniqueTest<BulletTest>);

    BulletTest()
    {
        {
            const auto body = CreateBody(GetWorld(), BodyConf{}.UseLocation(Length2{}));
            Attach(GetWorld(), body,
                   CreateShape(GetWorld(),
                               EdgeShapeConf{Vec2(-10.0f, 0.0f) * 1_m, Vec2(10.0f, 0.0f) * 1_m}));
            Attach(GetWorld(), body,
                   CreateShape(GetWorld(), PolygonShapeConf{}.SetAsBox(
                                               0.2_m, 1_m, Vec2(0.5f, 1.0f) * 1_m, 0_rad)));
        }

        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.linearAcceleration = GetGravity();
            bd.UseLocation(Vec2(0.0f, 4.0f) * 1_m);

            auto conf = PolygonShapeConf{};
            conf.UseDensity(1_kgpm2);
            conf.SetAsBox(2_m, 0.1_m);

            m_body = CreateBody(GetWorld(), bd);
            Attach(GetWorld(), m_body, CreateShape(GetWorld(), conf));

            conf.UseDensity(100_kgpm2);
            conf.SetAsBox(0.25_m, 0.25_m);

            // m_x = RandomFloat(-1.0f, 1.0f);
            m_x = 0.20352793f;
            bd.UseLocation(Vec2(m_x, 10.0f) * 1_m);
            bd.bullet = true;

            m_bullet = CreateBody(GetWorld(), bd);
            Attach(GetWorld(), m_bullet, CreateShape(GetWorld(), conf));

            SetVelocity(GetWorld(), m_bullet, Velocity{Vec2{0.0f, -50.0f} * 1_mps, 0_rpm});
        }
    }

    void Launch()
    {
        SetTransform(GetWorld(), m_body, Vec2(0.0f, 4.0f) * 1_m, 0_rad);
        SetVelocity(GetWorld(), m_body, Velocity{LinearVelocity2{}, 0_rpm});

        m_x = RandomFloat(-1.0f, 1.0f);
        SetTransform(GetWorld(), m_bullet, Vec2(m_x, 10.0f) * 1_m, 0_rad);
        SetVelocity(GetWorld(), m_bullet, Velocity{Vec2(0.0f, -50.0f) * 1_mps, 0_rpm});
    }

    void PostStep(const Settings&, Drawer&) override
    {
        if (GetStepCount() % 60 == 0) {
            Launch();
        }
    }

    BodyID m_body;
    BodyID m_bullet;
    Real m_x;
};

} // namespace testbed
