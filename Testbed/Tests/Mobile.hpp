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

#ifndef PLAYRHO_MOBILE_HPP
#define PLAYRHO_MOBILE_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class Mobile : public Test
{
public:

    enum
    {
        e_depth = 4
    };

    Mobile()
    {
        // Create ground body.
        const auto ground = m_world.CreateBody(BodyConf{}.UseLocation(Vec2(0.0f, 20.0f) * 1_m));

        const auto a = Real{0.5f};
        const auto shape = Shape(PolygonShapeConf{}.UseDensity(20_kgpm2).SetAsBox(Real{0.25f} * a * 1_m, a * 1_m));

        RevoluteJointConf jointConf;
        jointConf.bodyA = ground;
        jointConf.bodyB = AddNode(ground, Length2{}, 0, 3.0f, static_cast<float>(a), shape);
        jointConf.localAnchorA = Length2{};
        jointConf.localAnchorB = Vec2(0, a) * 1_m;
        m_world.CreateJoint(jointConf);
    }

    Body* AddNode(Body* parent, Length2 localAnchor, int depth, float offset, float a,
                  Shape shape)
    {
        const auto h = Vec2(0.0f, a) * 1_m;

        BodyConf bodyConf;
        bodyConf.type = BodyType::Dynamic;
        bodyConf.location = parent->GetLocation() + localAnchor - h;
        const auto body = m_world.CreateBody(bodyConf);
        body->CreateFixture(shape);

        if (depth == e_depth)
        {
            return body;
        }

        const auto a1 = Vec2(offset, -a) * 1_m;
        const auto a2 = Vec2(-offset, -a) * 1_m;

        RevoluteJointConf jointConf;
        jointConf.bodyA = body;
        jointConf.localAnchorB = h;

        jointConf.localAnchorA = a1;
        jointConf.bodyB = AddNode(body, a1, depth + 1, 0.5f * offset, a, shape);
        m_world.CreateJoint(jointConf);

        jointConf.localAnchorA = a2;
        jointConf.bodyB = AddNode(body, a2, depth + 1, 0.5f * offset, a, shape);
        m_world.CreateJoint(jointConf);

        return body;
    }
};

} // namespace testbed

#endif
