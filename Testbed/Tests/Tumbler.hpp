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

    static constexpr auto Count = 800;

    enum class ShapeType
    {
        Square, Disk
    };
    
    static Body* CreateEnclosure(World& world)
    {
        const auto b = world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic)
                                        .UseLocation(Vec2(0, 10) * 1_m)
                                        .UseAllowSleep(false));
        
        PolygonShape shape;
        shape.SetDensity(5_kgpm2);
        SetAsBox(shape, 0.5_m, 10_m, Vec2( 10.0f, 0.0f) * 1_m, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, 0.5_m, 10_m, Vec2(-10.0f, 0.0f) * 1_m, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, 10_m, 0.5_m, Vec2(0.0f, 10.0f) * 1_m, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, 10_m, 0.5_m, Vec2(0.0f, -10.0f) * 1_m, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));

        return b;
    }

    static RevoluteJoint* CreateRevoluteJoint(World& world, Body* stable, Body* turn)
    {
        RevoluteJointDef jd;
        jd.bodyA = stable;
        jd.bodyB = turn;
        jd.localAnchorA = Vec2(0.0f, 10.0f) * 1_m;
        jd.localAnchorB = Vec2(0.0f, 0.0f) * 1_m;
        jd.referenceAngle = Angle{0};
        jd.motorSpeed = Pi * 0.05_rad / 1_s;
        jd.maxMotorTorque = 100000_Nm; // 1e8f;
        jd.enableMotor = true;
        return static_cast<RevoluteJoint*>(world.CreateJoint(jd));
    }
    
    Tumbler()
    {
        m_square->SetDensity(1_kgpm2);
        m_disk->SetDensity(0.1_kgpm2);

        const auto g = m_world.CreateBody(BodyDef{}.UseType(BodyType::Static));
        const auto b = CreateEnclosure(m_world);
        m_joint = CreateRevoluteJoint(m_world, g, b);
        
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "Speed up rotation.", [&](KeyActionMods) {
            m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() + Pi * 0.01_rad / 1_s);
        });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "Slow down rotation.", [&](KeyActionMods) {
            m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() - Pi * 0.01_rad / 1_s);
        });
        RegisterForKey(GLFW_KEY_EQUAL, GLFW_PRESS, 0, "Stop rotation.", [&](KeyActionMods) {
            m_joint->SetMotorSpeed(0_rad / 1_s);
        });
        RegisterForKey(GLFW_KEY_0, GLFW_PRESS, 0, "for remaining emitted shapes to be disks.", [&](KeyActionMods) {
            m_shapeType = ShapeType::Disk;
        });
        RegisterForKey(GLFW_KEY_1, GLFW_PRESS, 0, "for remaining emitted shapes to be squares.", [&](KeyActionMods) {
            m_shapeType = ShapeType::Square;
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

    Body* CreateBody()
    {
        return m_world.CreateBody(BodyDef{}
                            .UseType(BodyType::Dynamic)
                            .UseLocation(Vec2(0, 10) * 1_m)
                            .UseUserData(reinterpret_cast<void*>(1)));
    }

    void AddSquare()
    {
        CreateBody()->CreateFixture(m_square);
    }
    
    void AddDisk()
    {
        CreateBody()->CreateFixture(m_disk);
    }

    void PostStep(const Settings& settings, Drawer&) override
    {
        if ((!settings.pause || settings.singleStep) && (m_count < Count))
        {
            switch (m_shapeType)
            {
                case ShapeType::Square:
                    AddSquare();
                    break;
                case ShapeType::Disk:
                    AddDisk();
                    break;
            }
            ++m_count;
        }
    }

    RevoluteJoint* m_joint;
    ShapeType m_shapeType = ShapeType::Square;
    int m_count = 0;
    std::shared_ptr<PolygonShape> m_square = std::make_shared<PolygonShape>(0.125_m, 0.125_m);
    std::shared_ptr<DiskShape> m_disk = std::make_shared<DiskShape>(DiskShape::Conf{}
                                                                    .UseVertexRadius(0.125_m)
                                                                    .UseFriction(Real(0)));
};

} // namespace playrho

#endif
