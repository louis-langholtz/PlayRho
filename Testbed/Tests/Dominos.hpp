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

#ifndef PLAYRHO_DOMINOS_HPP
#define  PLAYRHO_DOMINOS_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class Dominos : public Test
{
public:

    Dominos()
    {
        const auto b1 = m_world->CreateBody();
        b1->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));

        {
            BodyDef bd;
            bd.position = Vec2(-1.5f, 10.0f) * Meter;
            const auto ground = m_world->CreateBody(bd);
            ground->CreateFixture(std::make_shared<PolygonShape>(PolygonShape{Real{6.0f} * Meter, Real{0.25f} * Meter}));
        }

        {
            const auto shape = std::make_shared<PolygonShape>(Real{0.1f} * Meter, Real{1.0f} * Meter);
            shape->SetDensity(Real{20} * KilogramPerSquareMeter);
            shape->SetFriction(Real(0.05f));

            for (auto i = 0; i < 10; ++i)
            {
                const auto body = m_world->CreateBody(BodyDef{}
                                                      .UseType(BodyType::Dynamic)
                                                      .UseLocation(Vec2(-6.0f + 1.0f * i, 11.25f) * Meter));
                body->CreateFixture(shape);
            }
        }

        {
            PolygonShape shape;
            SetAsBox(shape, Real{7.2f} * Meter, Real{0.25f} * Meter, Vec2_zero * Meter, Real{0.3f} * Radian);

            BodyDef bd;
            bd.position = Vec2(1.2f, 6.0f) * Meter;
            const auto ground = m_world->CreateBody(bd);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        const auto b2 = m_world->CreateBody(BodyDef{}.UseLocation(Vec2(-7.0f, 4.0f) * Meter));
        b2->CreateFixture(std::make_shared<PolygonShape>(Real{0.25f} * Meter, Real{1.5f} * Meter));

        Body* b3;
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(-0.9f, 1.0f) * Meter;
            bd.angle = Real{-0.15f} * Radian;

            b3 = m_world->CreateBody(bd);
            auto conf = PolygonShape::Conf{};
            conf.density = Real{10} * KilogramPerSquareMeter;
            b3->CreateFixture(std::make_shared<PolygonShape>(Real{6.0f} * Meter,
                                                             Real{0.125f} * Meter, conf));
        }

        m_world->CreateJoint(RevoluteJointDef{b1, b3, Vec2(-2.0f, 1.0f) * Meter}
                             .UseCollideConnected(true));

        Body* b4;
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(-10.0f, 15.0f) * Meter;
            b4 = m_world->CreateBody(bd);
            auto conf = PolygonShape::Conf{};
            conf.density = Real{10} * KilogramPerSquareMeter;
            b4->CreateFixture(std::make_shared<PolygonShape>(Real{0.25f} * Meter,
                                                             Real{0.25f} * Meter, conf));
        }

        m_world->CreateJoint(RevoluteJointDef{b2, b4, Vec2(-7.0f, 15.0f) * Meter}
                             .UseCollideConnected(true));

        Body* b5;
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(6.5f, 3.0f) * Meter;
            b5 = m_world->CreateBody(bd);

            auto conf = PolygonShape::Conf{};
            conf.density = Real{10} * KilogramPerSquareMeter;
            conf.friction = 0.1f;

            PolygonShape shape{conf};

            SetAsBox(shape, Real{1.0f} * Meter, Real{0.1f} * Meter, Vec2(0.0f, -0.9f) * Meter,
                     Real{0.0f} * Radian);
            b5->CreateFixture(std::make_shared<PolygonShape>(shape));

            SetAsBox(shape, Real{0.1f} * Meter, Real{1.0f} * Meter, Vec2(-0.9f, 0.0f) * Meter,
                     Real{0.0f} * Radian);
            b5->CreateFixture(std::make_shared<PolygonShape>(shape));

            SetAsBox(shape, Real{0.1f} * Meter, Real{1.0f} * Meter, Vec2(0.9f, 0.0f) * Meter,
                     Real{0.0f} * Radian);
            b5->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        m_world->CreateJoint(RevoluteJointDef{b1, b5, Vec2(6.0f, 2.0f) * Meter}
                             .UseCollideConnected(true));

        Body* b6;
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(6.5f, 4.1f) * Meter;
            b6 = m_world->CreateBody(bd);
            auto conf = PolygonShape::Conf{};
            conf.density = Real{30} * KilogramPerSquareMeter;
            b6->CreateFixture(std::make_shared<PolygonShape>(PolygonShape(Real{1.0f} * Meter,
                                                                          Real{0.1f} * Meter, conf)));
        }

        m_world->CreateJoint(RevoluteJointDef{b5, b6, Vec2(7.5f, 4.0f) * Meter}
                             .UseCollideConnected(true));

        Body* b7;
        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(7.4f, 1.0f) * Meter;

            b7 = m_world->CreateBody(bd);
            auto conf = PolygonShape::Conf{};
            conf.density = Real{10} * KilogramPerSquareMeter;
            b7->CreateFixture(std::make_shared<PolygonShape>(PolygonShape(Real{0.1f} * Meter,
                                                                          Real{1.0f} * Meter, conf)));
        }

        DistanceJointDef djd;
        djd.bodyA = b3;
        djd.bodyB = b7;
        djd.localAnchorA = Vec2(6.0f, 0.0f) * Meter;
        djd.localAnchorB = Vec2(0.0f, -1.0f) * Meter;
        const auto d = GetWorldPoint(*djd.bodyB, djd.localAnchorB) - GetWorldPoint(*djd.bodyA, djd.localAnchorA);
        djd.length = GetLength(d);
        m_world->CreateJoint(djd);

        {
            const auto radius = Real{0.2f} * Meter;
            auto conf = DiskShape::Conf{};
            conf.density = Real{10} * KilogramPerSquareMeter;
            conf.vertexRadius = radius;
            const auto shape = std::make_shared<DiskShape>(conf);
            for (auto i = 0; i < 4; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Length2D{
                    Real{5.9f} * Meter + Real{2.0f} * radius * static_cast<Real>(i),
                    Real{2.4f} * Meter
                };
                const auto body = m_world->CreateBody(bd);
                body->CreateFixture(shape);
            }
        }
    }
};

} // namespace playrho

#endif
