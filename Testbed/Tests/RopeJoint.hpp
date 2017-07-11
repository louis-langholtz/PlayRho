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

#ifndef ROPE_JOINT_H
#define ROPE_JOINT_H

#include "../Framework/Test.hpp"

namespace box2d {

/// This test shows how a rope joint can be used to stabilize a chain of
/// bodies with a heavy payload. Notice that the rope joint just prevents
/// excessive stretching and has no other effect.
/// By disabling the rope joint you can see that the Box2D solver has trouble
/// supporting heavy bodies with light bodies. Try playing around with the
/// densities, time step, and iterations to see how they affect stability.
/// This test also shows how to use contact filtering. Filtering is configured
/// so that the payload does not collide with the chain.
class RopeJointTest : public Test
{
public:
    RopeJointTest()
    {
        const auto ground = m_world->CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));

        {
            const auto rectangle = std::make_shared<PolygonShape>(Real{0.5f} * Meter, Real{0.125f} * Meter);
            rectangle->SetDensity(Real{20} * KilogramPerSquareMeter);
            rectangle->SetFriction(0.2f);

            const auto square = std::make_shared<PolygonShape>(Real{1.5f} * Meter, Real{1.5f} * Meter);
            square->SetDensity(Real{100} * KilogramPerSquareMeter);
            square->SetFriction(0.2f);

            FixtureDef fd;
            fd.filter.categoryBits = 0x0001;
            fd.filter.maskBits = 0xFFFF & ~0x0002;

            const auto N = 10;
            const auto y = 15.0f;
            m_ropeDef.localAnchorA = Vec2(0.0f, y) * Meter;

            auto prevBody = ground;
            for (auto i = 0; i < N; ++i)
            {
                auto shape = rectangle;
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2(0.5f + 1.0f * i, y) * Meter;
                if (i == N - 1)
                {
                    shape = square;
                    fd.filter.categoryBits = 0x0002;
                    bd.position = Vec2(1.0f * i, y) * Meter;
                    bd.angularDamping = Real(0.4f) * Hertz;
                }

                const auto body = m_world->CreateBody(bd);

                body->CreateFixture(shape, fd);

                m_world->CreateJoint(RevoluteJointDef{prevBody, body, Vec2(Real(i), y) * Meter});

                prevBody = body;
            }

            m_ropeDef.localAnchorB = Vec2_zero * Meter;

            const auto extraLength = 0.01f;
            m_ropeDef.maxLength = Real(N - 1.0f + extraLength) * Meter;
            m_ropeDef.bodyB = prevBody;
        }

        m_ropeDef.bodyA = ground;
        m_rope = m_world->CreateJoint(m_ropeDef);
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_J:
            if (m_rope)
            {
                m_world->Destroy(m_rope);
                m_rope = nullptr;
            }
            else
            {
                m_rope = m_world->CreateJoint(m_ropeDef);
            }
            break;

        default:
            break;
        }
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, "Press (j) to toggle the rope joint.");
        m_textLine += DRAW_STRING_NEW_LINE;
        if (m_rope)
        {
            drawer.DrawString(5, m_textLine, "Rope ON");
        }
        else
        {
            drawer.DrawString(5, m_textLine, "Rope OFF");
        }
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    RopeJointDef m_ropeDef;
    Joint* m_rope;
};

} // namespace box2d

#endif
