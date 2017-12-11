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

namespace playrho {

class Tumbler : public Test
{
public:
    static PLAYRHO_CONSTEXPR const auto Count = 800;
    
    Tumbler()
    {
        SetupTumblers(1);
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "Speed up rotation.", [&](KeyActionMods) {
            ForAll<RevoluteJoint>(m_world, [=](RevoluteJoint& j) { IncMotorSpeed(j, +MotorInc); });
        });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "Slow down rotation.", [&](KeyActionMods) {
            ForAll<RevoluteJoint>(m_world, [=](RevoluteJoint& j) { IncMotorSpeed(j, -MotorInc); });
        });
        RegisterForKey(GLFW_KEY_EQUAL, GLFW_PRESS, 0, "Stop rotation.", [&](KeyActionMods) {
            ForAll<RevoluteJoint>(m_world, [=](RevoluteJoint& j) { j.SetMotorSpeed(0_rpm); });
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
            std::vector<Body*> bodies;
            for (auto&& body: m_world.GetBodies())
            {
                auto& b = GetRef(body);
                if (b.GetUserData() == reinterpret_cast<void*>(1))
                {
                    bodies.push_back(&b);
                }
            }
            for (auto&& b: bodies)
            {
                m_world.Destroy(b);
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

    Body* CreateEnclosure(Length2 at)
    {
        const auto b = m_world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic)
                                          .UseLocation(at).UseAllowSleep(false));
        auto shape = PolygonShape::Conf{}.UseDensity(5_kgpm2);
        shape.SetAsBox(0.5_m, 10_m, Vec2( 10,   0) * 1_m, 0_rad);
        b->CreateFixture(Shape(shape));
        shape.SetAsBox(0.5_m, 10_m, Vec2(-10,   0) * 1_m, 0_rad);
        b->CreateFixture(Shape(shape));
        shape.SetAsBox(10_m, 0.5_m, Vec2(  0,  10) * 1_m, 0_rad);
        b->CreateFixture(Shape(shape));
        shape.SetAsBox(10_m, 0.5_m, Vec2(  0, -10) * 1_m, 0_rad);
        b->CreateFixture(Shape(shape));
        return b;
    }
    
    RevoluteJoint* CreateRevoluteJoint(Body* turn)
    {
        RevoluteJointDef jd;
        jd.bodyA = m_world.CreateBody(BodyDef{}.UseLocation(GetLocation(*turn)));
        jd.bodyB = turn;
        jd.referenceAngle = 0_rad;
        jd.motorSpeed = 1.5_rpm; // same as Pi*0.05_rad/s = 0.025 rev/s
        jd.maxMotorTorque = 100000_Nm; // 1e8f;
        jd.enableMotor = true;
        return static_cast<RevoluteJoint*>(m_world.CreateJoint(jd));
    }
    
    void CreateTumblee(Length2 at)
    {
        const auto b = m_world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(at)
                                          .UseUserData(reinterpret_cast<void*>(1)));
        b->CreateFixture(m_shape);
    }

    void PostStep(const Settings& settings, Drawer&) override
    {
        if ((!settings.pause || settings.singleStep) && (m_count < Count))
        {
            ForAll<RevoluteJoint>(m_world, [&](RevoluteJoint& j) {
                CreateTumblee(GetLocation(*j.GetBodyB()));
            });
            ++m_count;
            m_status = std::string("Count = ") + std::to_string(m_count);
        }
    }

    const AngularVelocity MotorInc = 0.5_rpm;
    int m_count = 0;
    Shape m_square = Shape{PolygonShape::Conf{}.SetAsBox(0.125_m, 0.125_m).SetDensity(1_kgpm2)};
    Shape m_disk = Shape{DiskShape::Conf{}.UseVertexRadius(0.125_m).UseFriction(Real(0)).SetDensity(0.1_kgpm2)};
    Shape m_shape = m_square;
};

} // namespace playrho

#endif
