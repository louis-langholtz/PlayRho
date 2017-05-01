/*
* Original work Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#ifndef PULLEYS_H
#define PULLEYS_H

namespace box2d {

class Pulleys : public Test
{
public:
    Pulleys()
    {
        const auto y = RealNum{16.0f};
        const auto L = RealNum{12.0f};
        const auto a = RealNum{1.0f};
        const auto b = RealNum{2.0f};

        const auto ground = m_world->CreateBody();
        {
            auto conf = CircleShape::Conf{};
            conf.vertexRadius = RealNum{2.0f} * Meter;
            conf.location = Vec2(-10.0f, y + b + L) * Meter;
            CircleShape circle(conf);
            ground->CreateFixture(std::make_shared<CircleShape>(circle));

            circle.SetLocation(Vec2(10.0f, y + b + L) * Meter);
            ground->CreateFixture(std::make_shared<CircleShape>(circle));
        }

        {
            const auto shape = std::make_shared<PolygonShape>(a * Meter, b * Meter);
            shape->SetDensity(RealNum{5} * KilogramPerSquareMeter);

            BodyDef bd;
            bd.type = BodyType::Dynamic;

            //bd.fixedRotation = true;
            bd.position = Vec2(-10.0f, y) * Meter;
            const auto body1 = m_world->CreateBody(bd);
            body1->CreateFixture(shape);

            bd.position = Vec2(10.0f, y) * Meter;
            const auto body2 = m_world->CreateBody(bd);
            body2->CreateFixture(shape);

            PulleyJointDef pulleyDef;
            const auto anchor1 = Vec2(-10.0f, y + b) * Meter;
            const auto anchor2 = Vec2(10.0f, y + b) * Meter;
            const auto groundAnchor1 = Vec2(-10.0f, y + b + L) * Meter;
            const auto groundAnchor2 = Vec2(10.0f, y + b + L) * Meter;
            pulleyDef.Initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5f);

            m_joint1 = (PulleyJoint*)m_world->CreateJoint(pulleyDef);
        }
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        const auto ratio = m_joint1->GetRatio();
        const auto L = GetCurrentLengthA(*m_joint1) + ratio * GetCurrentLengthB(*m_joint1);
        drawer.DrawString(5, m_textLine, "L1 + %4.2f * L2 = %4.2f", (float) ratio, double{L / Meter});
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    static Test* Create()
    {
        return new Pulleys;
    }

    PulleyJoint* m_joint1;
};

} // namespace box2d

#endif
