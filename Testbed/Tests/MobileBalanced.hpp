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

#ifndef PLAYRHO_MOBILE_BALANCED_HPP
#define PLAYRHO_MOBILE_BALANCED_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class MobileBalanced : public Test
{
public:

    static constexpr int MaxDepth = 4;

    const AreaDensity density = 20_kgpm2;

    MobileBalanced()
    {
        const auto ground = m_world.CreateBody(BodyConf{}.UseLocation(Vec2(0.0f, 20.0f) * 1_m));

        const auto a = 0.5_m;
        const auto h = Length2{0_m, a};
        const auto root = AddNode(ground, Length2{}, 0, 3.0f, a,
                                  PolygonShapeConf{}.UseDensity(density).SetAsBox(a / 4, a));

        auto jointConf = RevoluteJointConf{};
        jointConf.bodyA = ground;
        jointConf.bodyB = root;
        jointConf.localAnchorA = Length2{};
        jointConf.localAnchorB = h;
        m_world.CreateJoint(jointConf);
    }

    Body* AddNode(const Body* parent, const Length2 localAnchor, const int depth,
                  const Real offset, const Length a, Shape shape)
    {
        const auto h = Length2{0_m, a};
        const auto p = parent->GetLocation() + localAnchor - h;

        BodyConf bodyConf;
        bodyConf.type = BodyType::Dynamic;
        bodyConf.linearAcceleration = m_gravity;
        bodyConf.location = p;
        const auto body = m_world.CreateBody(bodyConf);

        body->CreateFixture(shape);

        if (depth == MaxDepth)
        {
            return body;
        }

        auto shape2 = PolygonShapeConf{};
        shape2.UseDensity(density);
        shape2.SetAsBox(offset * 1_m, a / 4, Length2{0_m, -a}, 0_rad);
        body->CreateFixture(Shape(shape2));

        const auto a1 = Length2{offset * 1_m, -a};
        const auto a2 = Length2{-offset * 1_m, -a};
        const auto body1 = AddNode(body, a1, depth + 1, offset / 2, a, shape);
        const auto body2 = AddNode(body, a2, depth + 1, offset / 2, a, shape);

        RevoluteJointConf jointConf;
        jointConf.bodyA = body;
        jointConf.localAnchorB = h;

        jointConf.localAnchorA = a1;
        jointConf.bodyB = body1;
        m_world.CreateJoint(jointConf);

        jointConf.localAnchorA = a2;
        jointConf.bodyB = body2;
        m_world.CreateJoint(jointConf);

        return body;
    }
};

} // namespace testbed

#endif
