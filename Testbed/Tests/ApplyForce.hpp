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

#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

#include "../Framework/Test.hpp"

namespace box2d {

class ApplyForce : public Test
{
public:
    ApplyForce()
    {
        m_world->SetGravity(LinearAcceleration2D{0, 0});

        const auto k_restitution = RealNum(0.4);

        Body* ground;
        {
            BodyDef bd;
            bd.position = Length2D(RealNum(0) * Meter, RealNum(20) * Meter);
            ground = m_world->CreateBody(bd);

            auto conf = EdgeShape::Conf{};
            conf.density = 0;
            conf.restitution = k_restitution;
            EdgeShape shape(conf);

            // Left vertical
            shape.Set(Vec2(-20.0f, -20.0f) * Meter, Vec2(-20.0f, 20.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            // Right vertical
            shape.Set(Vec2(20.0f, -20.0f) * Meter, Vec2(20.0f, 20.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            // Top horizontal
            shape.Set(Vec2(-20.0f, 20.0f) * Meter, Vec2(20.0f, 20.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            // Bottom horizontal
            shape.Set(Vec2(-20.0f, -20.0f) * Meter, Vec2(20.0f, -20.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
        }

        {
            Transformation xf1;
            xf1.q = UnitVec2{0.3524f * Pi * Radian};
            xf1.p = GetVec2(GetXAxis(xf1.q)) * Meter;

            Length2D vertices[3];
            vertices[0] = Transform(Length2D{Vec2(RealNum{-1}, RealNum{0}) * Meter}, xf1);
            vertices[1] = Transform(Vec2(RealNum{1}, RealNum{0}) * Meter, xf1);
            vertices[2] = Transform(Vec2(RealNum{0}, RealNum{0.5f}) * Meter, xf1);

            auto conf = PolygonShape::Conf{};

            conf.density = RealNum{4} * KilogramPerSquareMeter;
            const auto poly1 = PolygonShape(Span<const Length2D>(vertices, 3), conf);

            Transformation xf2;
            xf2.q = UnitVec2{-0.3524f * Pi * Radian};
            xf2.p = GetVec2(-GetXAxis(xf2.q)) * Meter;

            vertices[0] = Transform(Vec2(-1.0f, RealNum{0}) * Meter, xf2);
            vertices[1] = Transform(Vec2(1.0f, RealNum{0}) * Meter, xf2);
            vertices[2] = Transform(Vec2(RealNum{0}, 0.5f) * Meter, xf2);

            conf.density = RealNum{2} * KilogramPerSquareMeter;
            const auto poly2 = PolygonShape(Span<const Length2D>(vertices, 3), conf);

            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.angularDamping = RealNum(2) * Hertz;
            bd.linearDamping = RealNum(0.5f) * Hertz;

            bd.position = Vec2(0, 2) * Meter;
            bd.angle = Pi * Radian;
            bd.allowSleep = false;
            m_body = m_world->CreateBody(bd);
            m_body->CreateFixture(std::make_shared<PolygonShape>(poly1));
            m_body->CreateFixture(std::make_shared<PolygonShape>(poly2));
        }

        {
            auto conf = PolygonShape::Conf{};
            conf.density = RealNum{1} * KilogramPerSquareMeter;
            conf.friction = RealNum{0.3f};
            const auto shape = std::make_shared<PolygonShape>(RealNum{0.5f} * Meter, RealNum{0.5f} * Meter, conf);

            const auto gravity = LinearAcceleration{RealNum{10} * MeterPerSquareSecond};

            for (auto i = 0; i < 10; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2(RealNum{0}, 5.0f + 1.54f * i) * Meter;
                const auto body = m_world->CreateBody(bd);

                body->CreateFixture(shape);

                const auto I = GetLocalInertia(*body); // RotInertia: M * L^2 QP^-2
                const auto invMass = body->GetInvMass(); // InvMass: M^-1
                const auto mass = (invMass != InvMass{0})? Mass{RealNum{1} / invMass}: Mass{0}; // Mass: M

                // For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
                const auto radiusSquaredUnitless = RealNum{2} * I * invMass * SquareRadian / SquareMeter;
                const auto radius = Length{RealNum{std::sqrt(RealNum{radiusSquaredUnitless})} * Meter};

                FrictionJointDef jd;
                jd.localAnchorA = Vec2_zero * Meter;
                jd.localAnchorB = Vec2_zero * Meter;
                jd.bodyA = ground;
                jd.bodyB = body;
                jd.collideConnected = true;
                jd.maxForce = Force{mass * gravity};

                // Torque is L^2 M T^-2 QP^-1.
                jd.maxTorque = Torque{mass * radius * gravity / Radian};

                m_world->CreateJoint(jd);
            }
        }
    }

    void KeyboardDown(Key key)
    {
        switch (key)
        {
        case Key_W:
            {
                const auto lv = Length2D{Vec2{RealNum{0}, RealNum{-200}} * Meter};
                const auto f = Force2D{GetWorldVector(*m_body, lv) * Kilogram / (Second * Second)};
                const auto p = GetWorldPoint(*m_body, Vec2(RealNum{0}, RealNum{2}) * Meter);
                box2d::ApplyForce(*m_body, f, p);
            }
            break;

        case Key_A:
            {
                ApplyTorque(*m_body, RealNum{50} * NewtonMeter);
            }
            break;

        case Key_D:
            {
                ApplyTorque(*m_body, RealNum{-50} * NewtonMeter);
            }
            break;

        default:
            break;
        }
    }

    Body* m_body;
};

} // namespace box2d

#endif
