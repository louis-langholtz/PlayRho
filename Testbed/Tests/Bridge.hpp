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

#ifndef PLAYRHO_BRIDGE_HPP
#define  PLAYRHO_BRIDGE_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class Bridge : public Test
{
public:

    static PLAYRHO_CONSTEXPR const auto Count = 30;

    Bridge()
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(Shape(GetGroundEdgeConf()));

        {
            auto conf = PolygonShape::Conf{};
            conf.density = 20_kgpm2;
            conf.friction = 0.2f;
            conf.SetAsBox(0.5_m, 0.125_m);
            const auto shape = Shape{conf};
            auto prevBody = ground;
            for (auto i = 0; i < Count; ++i)
            {
                const auto body = m_world.CreateBody(BodyDef{}
                                                     .UseType(BodyType::Dynamic)
                                                     .UseLocation(Vec2(-14.5f + i, 5.0f) * 1_m));
                body->CreateFixture(shape);

                m_world.CreateJoint(RevoluteJointDef{prevBody, body, Vec2(-15.0f + i, 5.0f) * 1_m});

                if (i == (Count >> 1))
                {
                    m_middle = body;
                }
                prevBody = body;
            }

            m_world.CreateJoint(RevoluteJointDef{prevBody, ground, Vec2(-15.0f + Count, 5.0f) * 1_m});
        }

        const auto conf = PolygonShape::Conf{}.UseDensity(1_kgpm2).UseVertices({
            Vec2(-0.5f, 0.0f) * 1_m,
            Vec2(0.5f, 0.0f) * 1_m,
            Vec2(0.0f, 1.5f) * 1_m
        });
        const auto polyshape = Shape(conf);
        for (auto i = 0; i < 2; ++i)
        {
            const auto body = m_world.CreateBody(BodyDef{}
                                                 .UseType(BodyType::Dynamic)
                                                 .UseLocation(Vec2(-8.0f + 8.0f * i, 12.0f) * 1_m));
            body->CreateFixture(polyshape);
        }

        const auto diskShape = Shape{DiskShapeConf{}.UseDensity(1_kgpm2).SetRadius(0.5_m)};
        for (auto i = 0; i < 3; ++i)
        {
            const auto body = m_world.CreateBody(BodyDef{}
                                                 .UseType(BodyType::Dynamic)
                                                 .UseLocation(Vec2(-6.0f + 6.0f * i, 10.0f) * 1_m));
            body->CreateFixture(diskShape);
        }
    }

    Body* m_middle;
};

} // namespace playrho

#endif
