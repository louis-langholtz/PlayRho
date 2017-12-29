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

#ifndef PLAYRHO_WEB_HPP
#define PLAYRHO_WEB_HPP

#include "../Framework/Test.hpp"

namespace testbed {

// This tests distance joints, body destruction, and joint destruction.
class Web : public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "Demonstrates a soft distance joint.";
        return conf;
    }
    
    Web(): Test(GetTestConf())
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});

        {
            const auto shape = Shape{PolygonShapeConf{}.UseDensity(5_kgpm2).SetAsBox(0.5_m, 0.5_m)};

            BodyConf bd;
            bd.type = BodyType::Dynamic;

            bd.location = Vec2(-5.0f, 5.0f) * 1_m;
            m_bodies[0] = m_world.CreateBody(bd);
            m_bodies[0]->CreateFixture(shape);

            bd.location = Vec2(5.0f, 5.0f) * 1_m;
            m_bodies[1] = m_world.CreateBody(bd);
            m_bodies[1]->CreateFixture(shape);

            bd.location = Vec2(5.0f, 15.0f) * 1_m;
            m_bodies[2] = m_world.CreateBody(bd);
            m_bodies[2]->CreateFixture(shape);

            bd.location = Vec2(-5.0f, 15.0f) * 1_m;
            m_bodies[3] = m_world.CreateBody(bd);
            m_bodies[3]->CreateFixture(shape);

            DistanceJointConf jd;
            Length2 p1, p2, d;

            jd.frequency = 2_Hz;
            jd.dampingRatio = 0.0f;

            jd.bodyA = ground;
            jd.bodyB = m_bodies[0];
            jd.localAnchorA = Vec2(-10.0f, 0.0f) * 1_m;
            jd.localAnchorB = Vec2(-0.5f, -0.5f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[0] = m_world.CreateJoint(jd);

            jd.bodyA = ground;
            jd.bodyB = m_bodies[1];
            jd.localAnchorA = Vec2(10.0f, 0.0f) * 1_m;
            jd.localAnchorB = Vec2(0.5f, -0.5f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[1] = m_world.CreateJoint(jd);

            jd.bodyA = ground;
            jd.bodyB = m_bodies[2];
            jd.localAnchorA = Vec2(10.0f, 20.0f) * 1_m;
            jd.localAnchorB = Vec2(0.5f, 0.5f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[2] = m_world.CreateJoint(jd);

            jd.bodyA = ground;
            jd.bodyB = m_bodies[3];
            jd.localAnchorA = Vec2(-10.0f, 20.0f) * 1_m;
            jd.localAnchorB = Vec2(-0.5f, 0.5f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[3] = m_world.CreateJoint(jd);

            jd.bodyA = m_bodies[0];
            jd.bodyB = m_bodies[1];
            jd.localAnchorA = Vec2(0.5f, 0.0f) * 1_m;
            jd.localAnchorB = Vec2(-0.5f, 0.0f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[4] = m_world.CreateJoint(jd);

            jd.bodyA = m_bodies[1];
            jd.bodyB = m_bodies[2];
            jd.localAnchorA = Vec2(0.0f, 0.5f) * 1_m;
            jd.localAnchorB = Vec2(0.0f, -0.5f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[5] = m_world.CreateJoint(jd);

            jd.bodyA = m_bodies[2];
            jd.bodyB = m_bodies[3];
            jd.localAnchorA = Vec2(-0.5f, 0.0f) * 1_m;
            jd.localAnchorB = Vec2(0.5f, 0.0f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[6] = m_world.CreateJoint(jd);

            jd.bodyA = m_bodies[3];
            jd.bodyB = m_bodies[0];
            jd.localAnchorA = Vec2(0.0f, -0.5f) * 1_m;
            jd.localAnchorB = Vec2(0.0f, 0.5f) * 1_m;
            p1 = GetWorldPoint(*jd.bodyA, jd.localAnchorA);
            p2 = GetWorldPoint(*jd.bodyB, jd.localAnchorB);
            d = p2 - p1;
            jd.length = GetMagnitude(d);
            m_joints[7] = m_world.CreateJoint(jd);
        }
        
        RegisterForKey(GLFW_KEY_B, GLFW_PRESS, 0, "Delete a body.", [&](KeyActionMods) {
            for (auto i = 0; i < 4; ++i)
            {
                if (m_bodies[i])
                {
                    m_world.Destroy(m_bodies[i]);
                    m_bodies[i] = nullptr;
                    break;
                }
            }
        });
        RegisterForKey(GLFW_KEY_J, GLFW_PRESS, 0, "Delete a joint.", [&](KeyActionMods) {
            for (auto i = 0; i < 8; ++i)
            {
                if (m_joints[i])
                {
                    m_world.Destroy(m_joints[i]);
                    m_joints[i] = nullptr;
                    break;
                }
            }
        });
    }

    void JointDestroyed(Joint* joint) override
    {
        for (auto i = 0; i < 8; ++i)
        {
            if (m_joints[i] == joint)
            {
                m_joints[i] = nullptr;
                break;
            }
        }
    }

    Body* m_bodies[4];
    Joint* m_joints[8];
};

} // namespace testbed

#endif
