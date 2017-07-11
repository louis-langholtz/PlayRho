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

#ifndef BRIDGE_H
#define BRIDGE_H

#include "../Framework/Test.hpp"

namespace box2d {

class Bridge : public Test
{
public:

    static constexpr auto Count = 30;

    Bridge()
    {
        const auto ground = m_world->CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));

        {
            auto conf = PolygonShape::Conf{};
            conf.density = Real{20} * KilogramPerSquareMeter;
            conf.friction = 0.2f;
            const auto shape = std::make_shared<PolygonShape>(Real{0.5f} * Meter, Real{0.125f} * Meter, conf);
            auto prevBody = ground;
            for (auto i = 0; i < Count; ++i)
            {
                const auto body = m_world->CreateBody(BodyDef{}
                                                      .UseType(BodyType::Dynamic)
                                                      .UseLocation(Vec2(-14.5f + 1.0f * i, 5.0f) * Meter));
                body->CreateFixture(shape);

                m_world->CreateJoint(RevoluteJointDef{prevBody, body, Vec2(-15.0f + 1.0f * i, 5.0f) * Meter});

                if (i == (Count >> 1))
                {
                    m_middle = body;
                }
                prevBody = body;
            }

            m_world->CreateJoint(RevoluteJointDef{prevBody, ground, Vec2(-15.0f + 1.0f * Count, 5.0f) * Meter});
        }

        const auto polyshape = std::make_shared<PolygonShape>(PolygonShape::Conf{}
                                                              .UseDensity(Real{1} * KilogramPerSquareMeter));
        polyshape->Set({
            Vec2(-0.5f, 0.0f) * Meter,
            Vec2(0.5f, 0.0f) * Meter,
            Vec2(0.0f, 1.5f) * Meter
        });
        for (auto i = 0; i < 2; ++i)
        {
            const auto body = m_world->CreateBody(BodyDef{}
                                                  .UseType(BodyType::Dynamic)
                                                  .UseLocation(Vec2(-8.0f + 8.0f * i, 12.0f) * Meter));
            body->CreateFixture(polyshape);
        }

        const auto diskShape = std::make_shared<DiskShape>(DiskShape::Conf{}
                                                           .UseDensity(Real{1} * KilogramPerSquareMeter)
                                                           .UseVertexRadius(Real{0.5f} * Meter));
        for (auto i = 0; i < 3; ++i)
        {
            const auto body = m_world->CreateBody(BodyDef{}
                                                  .UseType(BodyType::Dynamic)
                                                  .UseLocation(Vec2(-6.0f + 6.0f * i, 10.0f) * Meter));
            body->CreateFixture(diskShape);
        }
    }

    Body* m_middle;
};

} // namespace box2d

#endif
