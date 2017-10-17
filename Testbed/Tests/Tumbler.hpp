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
                                        .UseLocation(Vec2(0, 10) * Meter)
                                        .UseAllowSleep(false));
        
        PolygonShape shape;
        shape.SetDensity(5 * KilogramPerSquareMeter);
        SetAsBox(shape, 0.5f * Meter, 10.0f * Meter, Vec2( 10.0f, 0.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, 0.5f * Meter, 10.0f * Meter, Vec2(-10.0f, 0.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, 10.0f * Meter, 0.5f * Meter, Vec2(0.0f, 10.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, 10.0f * Meter, 0.5f * Meter, Vec2(0.0f, -10.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));

        return b;
    }

    static RevoluteJoint* CreateRevoluteJoint(World& world, Body* stable, Body* turn)
    {
        RevoluteJointDef jd;
        jd.bodyA = stable;
        jd.bodyB = turn;
        jd.localAnchorA = Vec2(0.0f, 10.0f) * Meter;
        jd.localAnchorB = Vec2(0.0f, 0.0f) * Meter;
        jd.referenceAngle = Angle{0};
        jd.motorSpeed = 0.05f * Pi * RadianPerSecond;
        jd.maxMotorTorque = Real{100000} * NewtonMeter; // 1e8f;
        jd.enableMotor = true;
        return static_cast<RevoluteJoint*>(world.CreateJoint(jd));
    }
    
    Tumbler()
    {
        m_square->SetDensity(Real(1) * KilogramPerSquareMeter);
        m_disk->SetDensity(Real(0.1) * KilogramPerSquareMeter);

        const auto g = m_world.CreateBody(BodyDef{}.UseType(BodyType::Static));
        const auto b = CreateEnclosure(m_world);
        m_joint = CreateRevoluteJoint(m_world, g, b);
        
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "Speed up rotation.", [&](KeyActionMods) {
            m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() + 0.01f * Pi * RadianPerSecond);
        });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "Slow down rotation.", [&](KeyActionMods) {
            m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() - 0.01f * Pi * RadianPerSecond);
        });
        RegisterForKey(GLFW_KEY_EQUAL, GLFW_PRESS, 0, "Stop rotation.", [&](KeyActionMods) {
            m_joint->SetMotorSpeed(Real(0) * RadianPerSecond);
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
                            .UseLocation(Vec2(0, 10) * Meter)
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
    std::shared_ptr<PolygonShape> m_square = std::make_shared<PolygonShape>(Real{0.125f} * Meter,
                                                                            Real{0.125f} * Meter);
    std::shared_ptr<DiskShape> m_disk = std::make_shared<DiskShape>(DiskShape::Conf{}
                                                                    .UseVertexRadius(Real(0.125f) * Meter)
                                                                    .UseFriction(Real(0)));
};

} // namespace playrho

#endif
