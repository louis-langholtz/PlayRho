/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_DOMINOS_HPP
#define  PLAYRHO_DOMINOS_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class Dominos : public Test
{
public:

    Dominos()
    {
        const auto b1 = CreateBody(GetWorld());
        CreateFixture(GetWorld(), b1, Shape{EdgeShapeConf(Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m)});

        {
            BodyConf bd;
            bd.location = Vec2(-1.5f, 10.0f) * 1_m;
            const auto ground = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), ground, Shape{PolygonShapeConf{6_m, 0.25_m}});
        }

        {
            const auto shape = Shape{
                PolygonShapeConf{}.UseDensity(20_kgpm2).UseFriction(Real(0.05)).SetAsBox(0.1_m, 1_m)
            };
            for (auto i = 0; i < 10; ++i)
            {
                const auto body = CreateBody(GetWorld(), BodyConf{}
                                                      .UseType(BodyType::Dynamic)
                                                      .UseLocation(Vec2(-6.0f + 1.0f * i, 11.25f) * 1_m));
                CreateFixture(GetWorld(), body, shape);
            }
        }

        {
            auto shape = PolygonShapeConf{};
            shape.SetAsBox(7.2_m, 0.25_m, Length2{}, 0.3_rad);

            BodyConf bd;
            bd.location = Vec2(1.2f, 6.0f) * 1_m;
            const auto ground = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), ground, Shape(shape));
        }

        const auto b2 = CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(-7.0f, 4.0f) * 1_m));
        CreateFixture(GetWorld(), b2, Shape{PolygonShapeConf{}.SetAsBox(0.25_m, 1.5_m)});

        BodyID b3;
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-0.9f, 1.0f) * 1_m;
            bd.angle = -0.15_rad;
            b3 = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), b3, Shape{PolygonShapeConf{}.UseDensity(10_kgpm2).SetAsBox(6_m, 0.125_m)});
        }

        CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), b1, b3, Vec2(-2, 1) * 1_m).UseCollideConnected(true));

        BodyID b4;
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-10.0f, 15.0f) * 1_m;
            b4 = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), b4, Shape{PolygonShapeConf{}.UseDensity(10_kgpm2).SetAsBox(0.25_m, 0.25_m)});
        }

        CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), b2, b4, Vec2(-7, 15) * 1_m).UseCollideConnected(true));

        BodyID b5;
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(6.5f, 3.0f) * 1_m;
            b5 = CreateBody(GetWorld(), bd);
            auto conf = PolygonShapeConf{};
            conf.density = 10_kgpm2;
            conf.friction = 0.1f;
            conf.SetAsBox(1_m, 0.1_m, Vec2(0.0f, -0.9f) * 1_m, 0_rad);
            CreateFixture(GetWorld(), b5, Shape{conf});
            conf.SetAsBox(0.1_m, 1_m, Vec2(-0.9f, 0.0f) * 1_m, 0_rad);
            CreateFixture(GetWorld(), b5, Shape{conf});
            conf.SetAsBox(0.1_m, 1_m, Vec2(0.9f, 0.0f) * 1_m, 0_rad);
            CreateFixture(GetWorld(), b5, Shape{conf});
        }

        CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), b1, b5, Vec2(6, 2) * 1_m).UseCollideConnected(true));

        BodyID b6;
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(6.5f, 4.1f) * 1_m;
            b6 = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), b6, Shape{PolygonShapeConf(1_m, 0.1_m).UseDensity(30_kgpm2)});
        }

        CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), b5, b6, Vec2(7.5f, 4.0f) * 1_m)
                             .UseCollideConnected(true));

        BodyID b7;
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(7.4f, 1.0f) * 1_m;
            b7 = CreateBody(GetWorld(), bd);
            auto conf = PolygonShapeConf{};
            conf.density = 10_kgpm2;
            conf.SetAsBox(0.1_m, 1_m);
            CreateFixture(GetWorld(), b7, Shape{conf});
        }

        DistanceJointConf djd;
        djd.bodyA = b3;
        djd.bodyB = b7;
        djd.localAnchorA = Vec2(6.0f, 0.0f) * 1_m;
        djd.localAnchorB = Vec2(0.0f, -1.0f) * 1_m;
        const auto d = GetWorldPoint(GetWorld(), djd.bodyB, djd.localAnchorB) - GetWorldPoint(GetWorld(), djd.bodyA, djd.localAnchorA);
        djd.length = GetMagnitude(d);
        CreateJoint(GetWorld(), djd);

        {
            const auto radius = 0.2_m;
            auto conf = DiskShapeConf{};
            conf.density = 10_kgpm2;
            conf.vertexRadius = radius;
            const auto shape = Shape(conf);
            for (auto i = 0; i < 4; ++i)
            {
                BodyConf bd;
                bd.type = BodyType::Dynamic;
                bd.location = Length2{5.9_m + 2 * radius * static_cast<Real>(i), 2.4_m};
                const auto body = CreateBody(GetWorld(), bd);
                CreateFixture(GetWorld(), body, shape);
            }
        }
        
        SetAccelerations(GetWorld(), GetGravity());
    }
};

} // namespace testbed

#endif
