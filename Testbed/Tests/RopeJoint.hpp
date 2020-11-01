/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_TESTS_ROPE_JOINT_HPP
#define PLAYRHO_TESTS_ROPE_JOINT_HPP

#include "../Framework/Test.hpp"

namespace testbed {

/// This test shows how a rope joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the rope joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the rope joint you can see that the solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
class RopeJointTest : public Test
{
public:
    RopeJointTest()
    {
        const auto ground = CreateBody(GetWorld());
        CreateFixture(GetWorld(), ground, Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});

        {
            const auto rectangle = Shape{
                PolygonShapeConf{}.UseDensity(20_kgpm2).UseFriction(0.2).SetAsBox(0.5_m, 0.125_m)
            };
            const auto square = Shape{
                PolygonShapeConf{}.UseDensity(100_kgpm2).UseFriction(0.2).SetAsBox(1.5_m, 1.5_m)
            };

            FixtureConf fd;
            fd.filter.categoryBits = 0x0001;
            fd.filter.maskBits = 0xFFFF & ~0x0002;

            const auto N = 10;
            const auto y = 15.0f;
            m_ropeConf.localAnchorA = Vec2(0.0f, y) * 1_m;

            auto prevBody = ground;
            for (auto i = 0; i < N; ++i)
            {
                auto shape = rectangle;
                BodyConf bd;
                bd.type = BodyType::Dynamic;
                bd.linearAcceleration = GetGravity();
                bd.location = Vec2(0.5f + 1.0f * i, y) * 1_m;
                if (i == N - 1)
                {
                    shape = square;
                    fd.filter.categoryBits = 0x0002;
                    bd.location = Vec2(1.0f * i, y) * 1_m;
                    bd.angularDamping = 0.4_Hz;
                }

                const auto body = CreateBody(GetWorld(), bd);

                CreateFixture(GetWorld(), body, shape, fd);

                CreateJoint(GetWorld(), GetRevoluteJointConf(GetWorld(), prevBody, body,
                                                          Vec2(Real(i), y) * 1_m));

                prevBody = body;
            }

            m_ropeConf.localAnchorB = Length2{};

            const auto extraLength = 0.01f;
            m_ropeConf.maxLength = Real(N - 1.0f + extraLength) * 1_m;
            m_ropeConf.bodyB = prevBody;
        }

        m_ropeConf.bodyA = ground;
        m_rope = CreateJoint(GetWorld(), m_ropeConf);
        
        RegisterForKey(GLFW_KEY_J, GLFW_PRESS, 0, "Toggle the rope joint", [&](KeyActionMods) {
            if (IsValid(m_rope))
            {
                Destroy(GetWorld(), m_rope);
                m_rope = InvalidJointID;
            }
            else
            {
                m_rope = CreateJoint(GetWorld(), m_ropeConf);
            }
        });
    }

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        stream << (IsValid(m_rope)? "Rope ON.": "Rope OFF.");
        SetStatus(stream.str());
    }

    RopeJointConf m_ropeConf;
    JointID m_rope;
};

} // namespace testbed

#endif /* PLAYRHO_TESTS_ROPE_JOINT_HPP */
