/*
* Original work Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_CAR_HPP
#define  PLAYRHO_CAR_HPP

#include "../Framework/Test.hpp"
#include <sstream>

namespace testbed {

// This is a demo that shows off the wheel joint and reflection transformations.
class Car : public Test
{
public:
    Car()
    {
        const auto motorSpeed = 50_rad / 1_s;
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Brake.", [&](KeyActionMods) {
            SetMotorSpeed(m_world, m_backSpring, 0_rpm);
        });
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Move Left.", [&](KeyActionMods) {
            SetMotorSpeed(m_world, m_backSpring, motorSpeed);
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Move Right.", [&](KeyActionMods) {
            SetMotorSpeed(m_world, m_backSpring, -motorSpeed);
        });
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, GLFW_MOD_SHIFT, "Turn Left.", [&](KeyActionMods) {
            CreateCar(true);
            SetMotorSpeed(m_world, m_backSpring, motorSpeed);
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, GLFW_MOD_SHIFT, "Turn Right.", [&](KeyActionMods) {
            CreateCar(false);
            SetMotorSpeed(m_world, m_backSpring, -motorSpeed);
        });
        RegisterForKey(GLFW_KEY_Q, GLFW_PRESS, 0, "Decrease Frequency.", [&](KeyActionMods) {
            m_hz = std::max(0_Hz, m_hz - 1_Hz);
            SetFrequency(m_world, m_backSpring, m_hz);
            SetFrequency(m_world, m_frontSpring, m_hz);
        });
        RegisterForKey(GLFW_KEY_E, GLFW_PRESS, 0, "Increase Frequency.", [&](KeyActionMods) {
            m_hz += 1_Hz;
            SetFrequency(m_world, m_backSpring, m_hz);
            SetFrequency(m_world, m_frontSpring, m_hz);
        });
        
        const auto ground = CreateBody(m_world);
        {
            const auto hs = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};
            auto x = Real{20.0f};
            auto y1 = Real{0.0f};
            const auto dx = decltype(x){5.0f};
            auto conf = EdgeShapeConf{}.UseDensity(0_kgpm2).UseFriction(Real(0.6));
            CreateFixture(m_world, ground, Shape{conf.Set(Vec2(-20, 0) * 1_m, Vec2(20, 0) * 1_m)});
            for (const auto y2: hs)
            {
                CreateFixture(m_world, ground, Shape{conf.Set(Vec2(x, y1) * 1_m, Vec2(x + dx, y2) * 1_m)});
                y1 = y2;
                x += dx;
            }
            for (const auto y2: hs)
            {
                CreateFixture(m_world, ground, Shape{conf.Set(Vec2(x, y1) * 1_m, Vec2(x + dx, y2) * 1_m)});
                y1 = y2;
                x += dx;
            }
            CreateFixture(m_world, ground, Shape{conf.Set(Vec2(x, 0) * 1_m, Vec2(x + 40, 0) * 1_m)});
            x += 80.0f;
            CreateFixture(m_world, ground, Shape{conf.Set(Vec2(x, 0) * 1_m, Vec2(x + 40, 0) * 1_m)});
            x += 40.0f;
            CreateFixture(m_world, ground, Shape{conf.Set(Vec2(x, 0) * 1_m, Vec2(x + 10, 5) * 1_m)});
            x += 20.0f;
            CreateFixture(m_world, ground, Shape{conf.Set(Vec2(x, 0) * 1_m, Vec2(x + 40, 0) * 1_m)});
            x += 40.0f;
            CreateFixture(m_world, ground, Shape{conf.Set(Vec2(x, 0) * 1_m, Vec2(x, 20) * 1_m)});
        }

        // Teeter
        {
            auto bd = BodyConf{};
            bd.location = Vec2(140.0f, 1.0f) * 1_m;
            bd.type = BodyType::Dynamic;
            const auto body = CreateBody(m_world, bd);
            CreateFixture(m_world, body, Shape{PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(10_m, 0.25_m)});

            auto jd = GetRevoluteJointConf(m_world, ground, body, GetLocation(m_world, body));
            jd.lowerAngle = -8_deg;
            jd.upperAngle = +8_deg;
            jd.enableLimit = true;
            m_world.CreateJoint(jd);

            // AngularMomentum is L^2 M T^-1 QP^-1.
            ApplyAngularImpulse(m_world, body, 100_m2 * 1_kg / (1_s * 1_rad));
        }

        // Bridge
        {
            const auto N = 20;
            const auto shape = Shape{
                PolygonShapeConf{}.UseDensity(1_kgpm2).UseFriction(Real(0.6)).SetAsBox(1_m, 0.125_m)
            };
            auto prevBody = ground;
            for (auto i = 0; i < N; ++i)
            {
                auto bd = BodyConf{}.UseType(BodyType::Dynamic);
                bd.location = Vec2(161 + 2 * i, -0.125f) * 1_m;
                const auto body = CreateBody(m_world, bd);
                CreateFixture(m_world, body, shape);
                m_world.CreateJoint(GetRevoluteJointConf(m_world, prevBody, body,
                                                         Vec2(160 + 2 * i, -0.125f) * 1_m));
                prevBody = body;
            }
            m_world.CreateJoint(GetRevoluteJointConf(m_world, prevBody, ground,
                                                     Vec2(160 + 2 * N, -0.125f) * 1_m));
        }

        // Boxes
        {
            const auto box = Shape{PolygonShapeConf{}.UseDensity(0.5_kgpm2).SetAsBox(0.5_m, 0.5_m)};
            auto bd = BodyConf{}.UseType(BodyType::Dynamic);
            CreateFixture(m_world, CreateBody(m_world, bd.UseLocation(Vec2(230, 0.5f) * 1_m)), box);
            CreateFixture(m_world, CreateBody(m_world, bd.UseLocation(Vec2(230, 1.5f) * 1_m)), box);
            CreateFixture(m_world, CreateBody(m_world, bd.UseLocation(Vec2(230, 2.5f) * 1_m)), box);
            CreateFixture(m_world, CreateBody(m_world, bd.UseLocation(Vec2(230, 3.5f) * 1_m)), box);
            CreateFixture(m_world, CreateBody(m_world, bd.UseLocation(Vec2(230, 4.5f) * 1_m)), box);
        }

        CreateCar();
        SetAccelerations(m_world, m_gravity);
    }

    void CreateCar(bool flip = false)
    {
        const auto carPosition = (m_car != InvalidBodyID)
            ? GetPosition(m_world, m_car)
            : Position{Length2{0_m, 1_m}, 0_deg};
        const auto carVelocity = (m_car != InvalidBodyID)
            ? GetVelocity(m_world, m_car): Velocity{};
        if (m_frontSpring != InvalidJointID) m_world.Destroy(GetBodyB(m_world, m_frontSpring));
        if (m_backSpring != InvalidJointID) m_world.Destroy(GetBodyB(m_world, m_backSpring));
        if (m_car != InvalidBodyID) m_world.Destroy(m_car);

        const auto transmat = flip? GetReflectionMatrix(UnitVec::GetRight()): GetIdentity<Mat22>();
        const auto carShapeConf = PolygonShapeConf{}.UseDensity(1_kgpm2).UseVertices({
            Vec2(-1.5f, -0.5f) * 1_m /* bottom left of car body */,
            Vec2(1.5f, -0.5f) * 1_m  /* bottom right of car body */,
            Vec2(1.5f, 0.0f) * 1_m   /* top right of car engine front */,
            Vec2(0.0f, 0.9f) * 1_m   /* top right of car roof */,
            Vec2(-1.15f, 0.9f) * 1_m /* top left of car roof */,
            Vec2(-1.5f, 0.2f) * 1_m  /* top left of car body */
        }).Transform(transmat);

        auto bd = BodyConf{}.UseType(BodyType::Dynamic).UseLinearAcceleration(m_gravity);
        m_car = CreateBody(m_world, bd.Use(carPosition).Use(carVelocity));
        CreateFixture(m_world, m_car, Shape{carShapeConf});
        
        const auto wheelShape = Shape{
            DiskShapeConf{}.UseRadius(0.4_m).UseDensity(1_kgpm2).UseFriction(Real(0.9))
        };
        {
            // setup back wheel
            const auto location = carPosition.linear + Rotate(transmat * Length2{-1_m, -0.65_m}, UnitVec::Get(carPosition.angular));
            const auto wheel = CreateBody(m_world, bd.UseLocation(location));
            CreateFixture(m_world, wheel, wheelShape);
            auto jd = GetWheelJointConf(m_world, m_car, wheel, GetLocation(m_world, wheel),
                                        UnitVec::GetTop());
            jd.maxMotorTorque = 20_Nm;
            jd.enableMotor = true;
            jd.frequency = m_hz;
            jd.dampingRatio = m_zeta;
            m_backSpring = m_world.CreateJoint(jd);
        }
        {
            // setup front wheel
            const auto location = carPosition.linear + Rotate(transmat * Length2{+1_m, -0.6_m}, UnitVec::Get(carPosition.angular));
            const auto wheel = CreateBody(m_world, bd.UseLocation(location));
            CreateFixture(m_world, wheel, wheelShape);
            auto jd = GetWheelJointConf(m_world, m_car, wheel, GetLocation(m_world, wheel),
                                        UnitVec::GetTop());
            jd.maxMotorTorque = 10_Nm;
            jd.enableMotor = false;
            jd.frequency = m_hz;
            jd.dampingRatio = m_zeta;
            m_frontSpring = m_world.CreateJoint(jd);
        }
    }
    
    Length2 DestroyCar()
    {
        const auto location = GetLocation(m_world, m_car);
        m_world.Destroy(GetBodyB(m_world, m_backSpring));
        m_world.Destroy(GetBodyB(m_world, m_frontSpring));
        m_world.Destroy(m_car);
        return location;
    }
    
    void PreStep(const Settings&, Drawer& drawer) override
    {
        drawer.SetTranslation(Length2{GetX(GetLocation(m_world, m_car)), GetY(drawer.GetTranslation())});
    }

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        stream << "Frequency = " << static_cast<double>(Real{m_hz / 1_Hz}) << " hz, ";
        stream << "damping ratio = " << m_zeta;
        m_status = stream.str();
    }

    const Real m_zeta = 0.7f;
    Frequency m_hz = 4_Hz;
    BodyID m_car = InvalidBodyID;
    JointID m_backSpring = InvalidJointID;
    JointID m_frontSpring = InvalidJointID;
};

} // namespace testbed

#endif
