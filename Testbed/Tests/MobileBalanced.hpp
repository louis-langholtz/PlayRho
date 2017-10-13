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

namespace playrho {

class MobileBalanced : public Test
{
public:

    enum
    {
        e_depth = 4
    };

    const Density density = Real{20} * KilogramPerSquareMeter;

    MobileBalanced()
    {
        const auto ground = m_world->CreateBody(BodyDef{}.UseLocation(Vec2(0.0f, 20.0f) * Meter));

        const auto a = Real{0.5f};
        const auto h = Vec2(0.0f, a) * Meter;

        auto conf = PolygonShape::Conf{};
        conf.density = density;
        const auto shape = std::make_shared<const PolygonShape>(Real{0.25f} * a * Meter, a * Meter, conf);
        const auto root = AddNode(ground, Vec2_zero * Meter, 0, 3.0f, a, shape);

        RevoluteJointDef jointDef;
        jointDef.bodyA = ground;
        jointDef.bodyB = root;
        jointDef.localAnchorA = Vec2_zero * Meter;
        jointDef.localAnchorB = h;
        m_world->CreateJoint(jointDef);
    }

    Body* AddNode(const Body* parent, const Length2D localAnchor, const int depth,
                  const Real offset, const Real a, std::shared_ptr<const Shape> shape)
    {
        const auto h = Vec2(0.0f, a) * Meter;

        const auto p = parent->GetLocation() + localAnchor - h;

        BodyDef bodyDef;
        bodyDef.type = BodyType::Dynamic;
        bodyDef.location = p;
        const auto body = m_world->CreateBody(bodyDef);

        body->CreateFixture(shape);

        if (depth == e_depth)
        {
            return body;
        }

        PolygonShape shape2(Real{0.25f} * a * Meter, Real{a} * Meter);
        shape2.SetDensity(density);
        SetAsBox(shape2, offset * Meter, Real{0.25f} * a * Meter, Vec2(0, -a) * Meter, Real{0.0f} * Radian);
        body->CreateFixture(std::make_shared<PolygonShape>(shape2));

        const auto a1 = Vec2(offset, -a) * Meter;
        const auto a2 = Vec2(-offset, -a) * Meter;
        const auto body1 = AddNode(body, a1, depth + 1, 0.5f * offset, a, shape);
        const auto body2 = AddNode(body, a2, depth + 1, 0.5f * offset, a, shape);

        RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.localAnchorB = h;

        jointDef.localAnchorA = a1;
        jointDef.bodyB = body1;
        m_world->CreateJoint(jointDef);

        jointDef.localAnchorA = a2;
        jointDef.bodyB = body2;
        m_world->CreateJoint(jointDef);

        return body;
    }
};

} // namespace playrho

#endif
