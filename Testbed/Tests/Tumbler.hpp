/*
 * Original work Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_TUMBLER_HPP
#define PLAYRHO_TUMBLER_HPP

#include "../Framework/Test.hpp"

#include <vector>

namespace testbed {

class Tumbler : public Test
{
public:
    static constexpr auto Count = 800;
    
    Tumbler()
    {
        SetupTumblers(1);
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "Speed up rotation.", [&](KeyActionMods) {
            for (const auto& id: GetJoints(m_world)) {
                if (GetType(m_world, id) == GetTypeID<RevoluteJointConf>()) {
                    SetMotorSpeed(m_world, id, GetMotorSpeed(m_world, id) + MotorInc);
                }
            }
        });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "Slow down rotation.", [&](KeyActionMods) {
            for (const auto& id: GetJoints(m_world)) {
                if (GetType(m_world, id) == GetTypeID<RevoluteJointConf>()) {
                    SetMotorSpeed(m_world, id, GetMotorSpeed(m_world, id) - MotorInc);
                }
            }
        });
        RegisterForKey(GLFW_KEY_EQUAL, GLFW_PRESS, 0, "Stop rotation.", [&](KeyActionMods) {
            for (const auto& id: GetJoints(m_world)) {
                if (GetType(m_world, id) == GetTypeID<RevoluteJointConf>()) {
                    SetMotorSpeed(m_world, id, 0_rpm);
                }
            }
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "for remaining emitted shapes to be disks.",
                       [&](KeyActionMods) { m_shape = m_disk; });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "for remaining emitted shapes to be squares.",
                       [&](KeyActionMods) { m_shape = m_square; });
        RegisterForKey(GLFW_KEY_1, GLFW_PRESS, 0, "Restart with 1 tumbler.", [&](KeyActionMods) {
            SetupTumblers(1);
        });
        RegisterForKey(GLFW_KEY_2, GLFW_PRESS, 0, "Restart with 2 tumblers.", [&](KeyActionMods) {
            SetupTumblers(2);
        });
        RegisterForKey(GLFW_KEY_C, GLFW_PRESS, 0, "Clear and re-emit shapes.", [&](KeyActionMods) {
            std::vector<BodyID> bodies;
            for (const auto& b: m_world.GetBodies()) {
                if ((b.get() < m_tumblee.size()) && m_tumblee[b.get()]) {
                    bodies.push_back(b);
                }
            }
            for (const auto& b: bodies) {
                Destroy(m_world, b);
            }
            m_count = 0;
        });
    }

    void SetupTumblers(unsigned int num)
    {
        m_world.Clear();
        m_count = 0;

        const auto width = 30_m;
        const auto halfWidth = width / 2;
        const auto totalWidth = num * width;
        auto ctrX = halfWidth - (totalWidth / 2);
        for (auto i = decltype(num){0}; i < num; ++i)
        {
            CreateRevoluteJoint(CreateEnclosure(Length2{ctrX, 20_m}));
            ctrX += width;
        }
    }

    BodyID CreateEnclosure(Length2 at)
    {
        const auto b = CreateBody(m_world, BodyConf{}.UseType(BodyType::Dynamic)
                                          .UseLocation(at).UseAllowSleep(false)
                                          .UseLinearAcceleration(GetGravity()));
        auto shape = PolygonShapeConf{}.UseDensity(5_kgpm2);
        shape.SetAsBox(0.5_m, 10_m, Vec2( 10,   0) * 1_m, 0_rad);
        CreateFixture(m_world, b, Shape(shape));
        shape.SetAsBox(0.5_m, 10_m, Vec2(-10,   0) * 1_m, 0_rad);
        CreateFixture(m_world, b, Shape(shape));
        shape.SetAsBox(10_m, 0.5_m, Vec2(  0,  10) * 1_m, 0_rad);
        CreateFixture(m_world, b, Shape(shape));
        shape.SetAsBox(10_m, 0.5_m, Vec2(  0, -10) * 1_m, 0_rad);
        CreateFixture(m_world, b, Shape(shape));
        return b;
    }

    JointID CreateRevoluteJoint(BodyID turn)
    {
        RevoluteJointConf jd;
        jd.bodyA = CreateBody(m_world, BodyConf{}.UseLocation(GetLocation(m_world, turn)));
        jd.bodyB = turn;
        jd.referenceAngle = 0_rad;
        jd.motorSpeed = 1.5_rpm; // same as Pi*0.05_rad/s = 0.025 rev/s
        jd.maxMotorTorque = 100000_Nm; // 1e8f;
        jd.enableMotor = true;
        return m_world.CreateJoint(jd);
    }

    void CreateTumblee(Length2 at)
    {
        const auto b = CreateBody(m_world, BodyConf{}.UseType(BodyType::Dynamic).UseLocation(at)
                                          .UseLinearAcceleration(GetGravity()));
        m_tumblee.resize(b.get() + 1u);
        m_tumblee[b.get()] = true;
        CreateFixture(m_world, b, m_shape);
    }

    void PostStep(const Settings& settings, Drawer&) override
    {
        if ((!settings.pause || settings.singleStep) && (m_count < Count))
        {
            for (const auto& id: GetJoints(m_world)) {
                if (GetType(m_world, id) == GetTypeID<RevoluteJointConf>()) {
                    CreateTumblee(GetLocation(m_world, GetBodyB(m_world, id)));
                }
            }
            ++m_count;
            SetStatus(std::string("Count = ") + std::to_string(m_count));
        }
    }

    const AngularVelocity MotorInc = 0.5_rpm;
    const Shape m_square = Shape{PolygonShapeConf{}.SetAsBox(0.125_m, 0.125_m).UseDensity(1_kgpm2)};
    const Shape m_disk = Shape{DiskShapeConf{}.UseRadius(0.125_m).UseFriction(0).UseDensity(0.1_kgpm2)};
    int m_count = 0;
    Shape m_shape = m_square;
    std::vector<bool> m_tumblee;
};

} // namespace testbed

#endif
