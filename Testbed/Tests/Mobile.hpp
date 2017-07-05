/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#ifndef MOBILE_H
#define MOBILE_H

#include "../Framework/Test.hpp"

namespace box2d {

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
        const auto ground = m_world->CreateBody(BodyDef{}.UseLocation(Vec2(0.0f, 20.0f) * Meter));

        const auto a = Real{0.5f};
        const auto shape = std::make_shared<PolygonShape>(Real{0.25f} * a * Meter, a * Meter);
        shape->SetDensity(Real{20} * KilogramPerSquareMeter);

        RevoluteJointDef jointDef;
        jointDef.bodyA = ground;
        jointDef.bodyB = AddNode(ground, Vec2_zero * Meter, 0, 3.0f, static_cast<float>(a), shape);
        jointDef.localAnchorA = Vec2_zero * Meter;
        jointDef.localAnchorB = Vec2(0, a) * Meter;
        m_world->CreateJoint(jointDef);
    }

    Body* AddNode(Body* parent, Length2D localAnchor, int depth, float offset, float a,
                  std::shared_ptr<Shape> shape)
    {
        const auto h = Vec2(0.0f, a) * Meter;

        BodyDef bodyDef;
        bodyDef.type = BodyType::Dynamic;
        bodyDef.position = parent->GetLocation() + localAnchor - h;
        const auto body = m_world->CreateBody(bodyDef);
        body->CreateFixture(shape);

        if (depth == e_depth)
        {
            return body;
        }

        const auto a1 = Vec2(offset, -a) * Meter;
        const auto a2 = Vec2(-offset, -a) * Meter;

        RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.localAnchorB = h;

        jointDef.localAnchorA = a1;
        jointDef.bodyB = AddNode(body, a1, depth + 1, 0.5f * offset, a, shape);
        m_world->CreateJoint(jointDef);

        jointDef.localAnchorA = a2;
        jointDef.bodyB = AddNode(body, a2, depth + 1, 0.5f * offset, a, shape);
        m_world->CreateJoint(jointDef);

        return body;
    }
};

} // namespace box2d

#endif
