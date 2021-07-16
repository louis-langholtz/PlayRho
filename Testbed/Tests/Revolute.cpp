/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "../Framework/Test.hpp"

namespace testbed {

class Revolute : public Test
{
public:
    static inline const auto registered = RegisterTest("Revolute", MakeUniqueTest<Revolute>);

    Revolute()
    {
        const auto ground = CreateBody(GetWorld());
        Attach(GetWorld(), ground,
               CreateShape(GetWorld(),
                           EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}));

        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;

            bd.location = Vec2(-10.0f, 20.0f) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);
            auto circleConf = DiskShapeConf{};
            circleConf.vertexRadius = 0.5_m;
            circleConf.density = 5_kgpm2;
            Attach(GetWorld(), body, CreateShape(GetWorld(), circleConf));

            const auto w = 100.0f;
            SetVelocity(GetWorld(), body, Velocity{Vec2(-8.0f * w, 0.0f) * 1_mps, w * 1_rad / 1_s});

            auto rjd = GetRevoluteJointConf(GetWorld(), ground, body, Vec2(-10.0f, 12.0f) * 1_m);
            rjd.motorSpeed = Pi * 1_rad / 1_s;
            rjd.maxMotorTorque = 10000_Nm;
            rjd.enableMotor = false;
            rjd.lowerAngle = -0.25_rad * Pi;
            rjd.upperAngle = 0.5_rad * Pi;
            rjd.enableLimit = true;
            rjd.collideConnected = true;

            m_joint = CreateJoint(GetWorld(), rjd);
        }

        {
            BodyConf circle_bd;
            circle_bd.type = BodyType::Dynamic;
            circle_bd.location = Vec2(5.0f, 30.0f) * 1_m;

            m_ball = CreateBody(GetWorld(), circle_bd);
            auto circleConf = DiskShapeConf{};
            circleConf.vertexRadius = 3_m;
            circleConf.density = 5_kgpm2;
            circleConf.filter.maskBits = 1;
            Attach(GetWorld(), m_ball, CreateShape(GetWorld(), circleConf));

            auto polygon_shape = PolygonShapeConf{};
            polygon_shape.SetAsBox(10_m, 0.2_m, Vec2(-10.0f, 0.0f) * 1_m, 0_rad);
            polygon_shape.UseDensity(2_kgpm2);

            BodyConf polygon_bd;
            polygon_bd.location = Vec2(20.0f, 10.0f) * 1_m;
            polygon_bd.type = BodyType::Dynamic;
            polygon_bd.bullet = true;
            const auto polygon_body = CreateBody(GetWorld(), polygon_bd);
            Attach(GetWorld(), polygon_body, CreateShape(GetWorld(), polygon_shape));

            auto rjd =
                GetRevoluteJointConf(GetWorld(), ground, polygon_body, Vec2(20.0f, 10.0f) * 1_m);
            rjd.lowerAngle = -0.25_rad * Pi;
            rjd.upperAngle = 0_rad * Pi;
            rjd.enableLimit = true;
            CreateJoint(GetWorld(), rjd);
        }

        // Tests mass computation of a small object far from the origin
        {
            const auto polyShape = PolygonShapeConf{}
                                       .Set({Vec2(17.63f, 36.31f) * 1_m, Vec2(17.52f, 36.69f) * 1_m,
                                             Vec2(17.19f, 36.36f) * 1_m})
                                       .UseDensity(1_kgpm2);

            const auto body = CreateBody(GetWorld(), BodyConf{}.UseType(BodyType::Dynamic));
            Attach(GetWorld(), body, CreateShape(GetWorld(), polyShape));
        }

        SetAccelerations(GetWorld(), GetGravity());

        RegisterForKey(GLFW_KEY_L, GLFW_PRESS, 0, "Limits", [&](KeyActionMods) {
            EnableLimit(GetWorld(), m_joint, !IsLimitEnabled(GetWorld(), m_joint));
        });
        RegisterForKey(GLFW_KEY_M, GLFW_PRESS, 0, "Motor", [&](KeyActionMods) {
            EnableMotor(GetWorld(), m_joint, !IsMotorEnabled(GetWorld(), m_joint));
        });
    }

    BodyID m_ball;
    JointID m_joint; // RevoluteJoint
};

} // namespace testbed
