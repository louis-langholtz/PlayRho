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

class Dominos : public Test
{
public:
    static inline const auto registered = RegisterTest("Dominos", MakeUniqueTest<Dominos>);

    Dominos()
    {
        const auto b1 = CreateBody(GetWorld());
        Attach(GetWorld(), b1,
               CreateShape(GetWorld(),
                           EdgeShapeConf(Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m)));

        {
            const auto ground = CreateBody(GetWorld(),
                                           BodyConf{}.UseLocation(Vec2(-1.5f, 10.0f) * 1_m));
            Attach(GetWorld(), ground, CreateShape(GetWorld(), PolygonShapeConf{6_m, 0.25_m}));
        }

        {
            const auto shape = CreateShape(GetWorld(), PolygonShapeConf{}
                                                           .UseDensity(20_kgpm2)
                                                           .UseFriction(Real(0.05))
                                                           .SetAsBox(0.1_m, 1_m));
            for (auto i = 0; i < 10; ++i) {
                const auto body =
                    CreateBody(GetWorld(), BodyConf{}
                                               .UseType(BodyType::Dynamic)
                                               .UseLocation(Vec2(-6.0f + 1.0f * i, 11.25f) * 1_m));
                Attach(GetWorld(), body, shape);
            }
        }

        {
            auto shape = PolygonShapeConf{};
            shape.SetAsBox(7.2_m, 0.25_m, Length2{}, 0.3_rad);
            const auto ground = CreateBody(GetWorld(),
                                           BodyConf{}.UseLocation(Vec2(1.2f, 6.0f) * 1_m));
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
        }

        const auto b2 = CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(-7.0f, 4.0f) * 1_m));
        Attach(GetWorld(), b2, CreateShape(GetWorld(), PolygonShapeConf{}.SetAsBox(0.25_m, 1.5_m)));

        BodyID b3;
        {
            b3 = CreateBody(GetWorld(),
                            BodyConf{}
                                .Use(BodyType::Dynamic)
                                .UseLocation(Vec2(-0.9f, 1.0f) * 1_m)
                                .UseAngle(-0.15_rad));
            Attach(GetWorld(), b3,
                   CreateShape(GetWorld(),
                               PolygonShapeConf{}.UseDensity(10_kgpm2).SetAsBox(6_m, 0.125_m)));
        }

        CreateJoint(
            GetWorld(),
            GetRevoluteJointConf(GetWorld(), b1, b3, Vec2(-2, 1) * 1_m).UseCollideConnected(true));

        BodyID b4;
        {
            b4 = CreateBody(GetWorld(),
                            BodyConf{}.Use(BodyType::Dynamic).UseLocation(Vec2(-10.0f, 15.0f) * 1_m));
            Attach(GetWorld(), b4,
                   CreateShape(GetWorld(),
                               PolygonShapeConf{}.UseDensity(10_kgpm2).SetAsBox(0.25_m, 0.25_m)));
        }

        CreateJoint(
            GetWorld(),
            GetRevoluteJointConf(GetWorld(), b2, b4, Vec2(-7, 15) * 1_m).UseCollideConnected(true));

        BodyID b5;
        {
            b5 = CreateBody(GetWorld(),
                            BodyConf{}.Use(BodyType::Dynamic).UseLocation(Vec2(6.5f, 3.0f) * 1_m));
            auto conf = PolygonShapeConf{};
            conf.density = 10_kgpm2;
            conf.friction = 0.1f;
            conf.SetAsBox(1_m, 0.1_m, Vec2(0.0f, -0.9f) * 1_m, 0_rad);
            Attach(GetWorld(), b5, CreateShape(GetWorld(), conf));
            conf.SetAsBox(0.1_m, 1_m, Vec2(-0.9f, 0.0f) * 1_m, 0_rad);
            Attach(GetWorld(), b5, CreateShape(GetWorld(), conf));
            conf.SetAsBox(0.1_m, 1_m, Vec2(0.9f, 0.0f) * 1_m, 0_rad);
            Attach(GetWorld(), b5, CreateShape(GetWorld(), conf));
        }

        CreateJoint(
            GetWorld(),
            GetRevoluteJointConf(GetWorld(), b1, b5, Vec2(6, 2) * 1_m).UseCollideConnected(true));

        BodyID b6;
        {
            b6 = CreateBody(GetWorld(),
                            BodyConf{}.Use(BodyType::Dynamic).UseLocation(Vec2(6.5f, 4.1f) * 1_m));
            Attach(GetWorld(), b6,
                   CreateShape(GetWorld(), PolygonShapeConf(1_m, 0.1_m).UseDensity(30_kgpm2)));
        }

        CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), b5, b6, Vec2(7.5f, 4.0f) * 1_m)
                                    .UseCollideConnected(true));

        BodyID b7;
        {
            b7 = CreateBody(GetWorld(),
                            BodyConf{}.Use(BodyType::Dynamic).UseLocation(Vec2(7.4f, 1.0f) * 1_m));
            auto conf = PolygonShapeConf{};
            conf.density = 10_kgpm2;
            conf.SetAsBox(0.1_m, 1_m);
            Attach(GetWorld(), b7, CreateShape(GetWorld(), conf));
        }

        DistanceJointConf djd;
        djd.bodyA = b3;
        djd.bodyB = b7;
        djd.localAnchorA = Vec2(6.0f, 0.0f) * 1_m;
        djd.localAnchorB = Vec2(0.0f, -1.0f) * 1_m;
        const auto d = GetWorldPoint(GetWorld(), djd.bodyB, djd.localAnchorB) -
                       GetWorldPoint(GetWorld(), djd.bodyA, djd.localAnchorA);
        djd.length = GetMagnitude(d);
        CreateJoint(GetWorld(), djd);

        {
            const auto r = 0.2_m;
            auto conf = DiskShapeConf{};
            conf.density = 10_kgpm2;
            conf.vertexRadius = r;
            const auto shape = CreateShape(GetWorld(), conf);
            for (auto i = 0; i < 4; ++i) {
                const auto body = CreateBody(GetWorld(),
                                             BodyConf{}
                                                 .Use(BodyType::Dynamic)
                                                 .UseLocation(Length2{5.9_m + 2 * r * Real(i), 2.4_m}));
                Attach(GetWorld(), body, shape);
            }
        }

        SetAccelerations(GetWorld(), GetGravity());
    }
};

} // namespace testbed
