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

#ifndef PLAYRHO_APPLY_FORCE_HPP
#define  PLAYRHO_APPLY_FORCE_HPP

#include "../Framework/Test.hpp"

namespace testbed {

class ApplyForce : public Test
{
public:
    
    ApplyForce()
    {
        m_world.SetGravity(LinearAcceleration2{});

        RegisterForKey(GLFW_KEY_W, GLFW_PRESS, 0, "Apply Force", [&](KeyActionMods) {
            const auto lv = Length2{0_m, -200_m};
            const auto f = Force2{GetWorldVector(*m_body, lv) * 1_kg / (1_s * 1_s)};
            const auto p = GetWorldPoint(*m_body, Length2{0_m, 2_m});
            playrho::ApplyForce(*m_body, f, p);
        });
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Apply Counter-Clockwise Torque", [&](KeyActionMods) {
            ApplyTorque(*m_body, 50_Nm);
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Apply Clockwise Torque", [&](KeyActionMods) {
            ApplyTorque(*m_body, -50_Nm);
        });

        const auto k_restitution = Real(0.4);

        Body* ground;
        {
            BodyDef bd;
            bd.location = Length2(0_m, 20_m);
            ground = m_world.CreateBody(bd);

            auto conf = EdgeShapeConf{};
            conf.density = 0;
            conf.restitution = k_restitution;

            // Left vertical
            conf.Set(Length2{-20_m, -20_m}, Length2{-20_m, 20_m});
            ground->CreateFixture(Shape{conf});

            // Right vertical
            conf.Set(Length2{20_m, -20_m}, Length2{20_m, 20_m});
            ground->CreateFixture(Shape{conf});

            // Top horizontal
            conf.Set(Length2{-20_m, 20_m}, Length2{20_m, 20_m});
            ground->CreateFixture(Shape{conf});

            // Bottom horizontal
            conf.Set(Length2{-20_m, -20_m}, Length2{20_m, -20_m});
            ground->CreateFixture(Shape{conf});
        }

        {
            Transformation xf1;
            xf1.q = UnitVec2::Get(0.3524_rad * Pi);
            xf1.p = GetVec2(GetXAxis(xf1.q)) * 1_m;

            Length2 vertices[3];
            vertices[0] = Transform(Length2{-1_m, 0_m}, xf1);
            vertices[1] = Transform(Length2{1_m, 0_m}, xf1);
            vertices[2] = Transform(Length2{0_m, 0.5_m}, xf1);

            auto conf = PolygonShapeConf{};

            conf.density = 4_kgpm2;
            const auto poly1 = PolygonShapeConf(Span<const Length2>(vertices, 3), conf);

            Transformation xf2;
            xf2.q = UnitVec2::Get(-0.3524_rad * Pi);
            xf2.p = GetVec2(-GetXAxis(xf2.q)) * 1_m;

            vertices[0] = Transform(Length2{-1_m, 0_m}, xf2);
            vertices[1] = Transform(Length2{1_m, 0_m}, xf2);
            vertices[2] = Transform(Length2{0_m, 0.5_m}, xf2);

            conf.density = 2_kgpm2;
            const auto poly2 = PolygonShapeConf(Span<const Length2>(vertices, 3), conf);

            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.angularDamping = 2_Hz;
            bd.linearDamping = 0.5_Hz;
            bd.location = Length2{0_m, 2_m};
            bd.angle = Pi * 1_rad;
            bd.allowSleep = false;
            m_body = m_world.CreateBody(bd);
            m_body->CreateFixture(Shape(poly1));
            m_body->CreateFixture(Shape(poly2));
        }

        {
            auto conf = PolygonShapeConf{};
            conf.density = 1_kgpm2;
            conf.friction = Real{0.3f};
            conf.SetAsBox(0.5_m, 0.5_m);
            const auto shape = Shape(conf);

            const auto gravity = LinearAcceleration{10_mps2};
            for (auto i = 0; i < 10; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Length2{0_m, (5.0f + 1.54f * i) * 1_m};
                const auto body = m_world.CreateBody(bd);

                body->CreateFixture(shape);

                const auto I = GetLocalRotInertia(*body); // RotInertia: M * L^2 QP^-2
                const auto invMass = body->GetInvMass(); // InvMass: M^-1
                const auto mass = (invMass != InvMass{0})? Mass{1 / invMass}: 0_kg;

                // For a circle: I = 0.5 * m * r * r ==> r = sqrt(2 * I / m)
                const auto radiusSquaredUnitless = 2 * I * invMass * SquareRadian / SquareMeter;
                const auto radius = Length{sqrt(Real{radiusSquaredUnitless}) * 1_m};

                FrictionJointDef jd;
                jd.localAnchorA = Length2{};
                jd.localAnchorB = Length2{};
                jd.bodyA = ground;
                jd.bodyB = body;
                jd.collideConnected = true;
                jd.maxForce = mass * gravity;

                // Torque is L^2 M T^-2 QP^-1.
                jd.maxTorque = mass * radius * gravity / 1_rad;

                m_world.CreateJoint(jd);
            }
        }
    }

    Body* m_body;
};

} // namespace testbed

#endif
