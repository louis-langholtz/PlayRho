/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_CANTILEVER_HPP
#define  PLAYRHO_CANTILEVER_HPP

#include "../Framework/Test.hpp"

namespace testbed {

// It is difficult to make a cantilever made of links completely rigid with weld joints.
// You will have to use a high number of iterations to make them stiff.
// So why not go ahead and use soft weld joints? They behave like a revolute
// joint with a rotational spring.
class Cantilever : public Test
{
public:

    enum
    {
        e_count = 8
    };

    Cantilever()
    {
        const auto ground = m_world.CreateBody();

        // Creates bottom ground
        m_world.CreateFixture(ground, Shape(GetGroundEdgeConf()));

        // Creates left-end-fixed 8-part plank (below the top one)
        {
            const auto shape = Shape{PolygonShapeConf{}.UseDensity(20_kgpm2).SetAsBox(0.5_m, 0.125_m)};
            auto prevBody = ground;
            for (auto i = 0; i < e_count; ++i)
            {
                auto bd = BodyConf{};
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(-14.5f + 1.0f * i, 5.0f) * 1_m;
                const auto body = m_world.CreateBody(bd);
                m_world.CreateFixture(body, shape);

                m_world.CreateJoint(GetWeldJointConf(m_world,
                    prevBody, body, Vec2(-15.0f + 1.0f * i, 5.0f) * 1_m
                ));

                prevBody = body;
            }
        }

        // Creates left-end-fixed 3-part plank at top
        {
            const auto shape = Shape{PolygonShapeConf{}.UseDensity(20_kgpm2).SetAsBox(1_m, 0.125_m)};
            auto prevBody = ground;
            for (auto i = 0; i < 3; ++i)
            {
                auto bd = BodyConf{};
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(-14.0f + 2.0f * i, 15.0f) * 1_m;
                const auto body = m_world.CreateBody(bd);
                m_world.CreateFixture(body, shape);

                auto jd = GetWeldJointConf(m_world, prevBody, body, Vec2(-15.0f + 2.0f * i, 15.0f) * 1_m);
                jd.frequency = 5_Hz;
                jd.dampingRatio = 0.7f;
                m_world.CreateJoint(jd);

                prevBody = body;
            }
        }

        // Creates 8-part plank to the right of the fixed planks (but not farthest right)
        {
            const auto shape = Shape{PolygonShapeConf{}.UseDensity(20_kgpm2).SetAsBox(0.5_m, 0.125_m)};
            auto prevBody = ground;
            for (auto i = 0; i < e_count; ++i)
            {
                auto bd = BodyConf{};
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(-4.5f + 1.0f * i, 5.0f) * 1_m;
                const auto body = m_world.CreateBody(bd);
                m_world.CreateFixture(body, shape);
                if (i > 0)
                {
                    m_world.CreateJoint(GetWeldJointConf(m_world,
                        prevBody, body, Vec2(-5.0f + 1.0f * i, 5.0f) * 1_m
                    ));
                }
                prevBody = body;
            }
        }

        // Creates 8-part farthest-right plank
        {
            const auto shape = Shape{PolygonShapeConf{}.UseDensity(20_kgpm2).SetAsBox(0.5_m, 0.125_m)};
            auto prevBody = ground;
            for (auto i = 0; i < e_count; ++i)
            {
                auto bd = BodyConf{};
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(5.5f + 1.0f * i, 10.0f) * 1_m;
                const auto body = m_world.CreateBody(bd);
                m_world.CreateFixture(body, shape);
                if (i > 0)
                {
                    auto jd = GetWeldJointConf(m_world, prevBody, body, Vec2(5.0f + 1.0f * i, 10.0f) * 1_m);
                    jd.frequency = 8_Hz;
                    jd.dampingRatio = 0.7f;
                    m_world.CreateJoint(jd);
                }
                prevBody = body;
            }
        }

        // Creates triangles
        const auto conf = PolygonShapeConf{}.UseDensity(1_kgpm2).UseVertices({
            Vec2(-0.5f, 0.0f) * 1_m, Vec2(0.5f, 0.0f) * 1_m, Vec2(0.0f, 1.5f) * 1_m});
        const auto polyshape = Shape{conf};
        for (auto i = 0; i < 2; ++i)
        {
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-8.0f + 8.0f * i, 12.0f) * 1_m;
            const auto body = m_world.CreateBody(bd);
            m_world.CreateFixture(body, polyshape);
        }

        // Creates circles
        const auto circleshape = Shape{DiskShapeConf{}.UseRadius(0.5_m).UseDensity(1_kgpm2)};
        for (auto i = 0; i < 2; ++i)
        {
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-6.0f + 6.0f * i, 10.0f) * 1_m;
            const auto body = m_world.CreateBody(bd);
            m_world.CreateFixture(body, circleshape);
        }
        
        SetAccelerations(m_world, m_gravity);
    }

    BodyID m_middle;
};

} // namespace testbed

#endif
