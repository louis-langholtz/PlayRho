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

#ifndef TUMBLER_H
#define TUMBLER_H

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

    Tumbler()
    {
        m_square->SetDensity(Real(1) * KilogramPerSquareMeter);
        m_disk->SetDensity(Real(0.1) * KilogramPerSquareMeter);

        const auto g = m_world->CreateBody(BodyDef{}.UseType(BodyType::Static));

        const auto b = m_world->CreateBody(BodyDef{}.UseType(BodyType::Dynamic)
                                           .UseLocation(Vec2(0, 10) * Meter)
                                           .UseAllowSleep(false));

        PolygonShape shape;
        shape.SetDensity(Real{5} * KilogramPerSquareMeter);
        SetAsBox(shape, Real{0.5f} * Meter, Real{10.0f} * Meter, Vec2( 10.0f, 0.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, Real{0.5f} * Meter, Real{10.0f} * Meter, Vec2(-10.0f, 0.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, Real{10.0f} * Meter, Real{0.5f} * Meter, Vec2(0.0f, 10.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));
        SetAsBox(shape, Real{10.0f} * Meter, Real{0.5f} * Meter, Vec2(0.0f, -10.0f) * Meter, Angle{0});
        b->CreateFixture(std::make_shared<PolygonShape>(shape));

        RevoluteJointDef jd;
        jd.bodyA = g;
        jd.bodyB = b;
        jd.localAnchorA = Vec2(0.0f, 10.0f) * Meter;
        jd.localAnchorB = Vec2(0.0f, 0.0f) * Meter;
        jd.referenceAngle = Angle{0};
        jd.motorSpeed = 0.05f * Pi * RadianPerSecond;
        jd.maxMotorTorque = Real{100000} * NewtonMeter; // 1e8f;
        jd.enableMotor = true;
        m_joint = static_cast<RevoluteJoint*>(m_world->CreateJoint(jd));
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine,
                          "Press C to clear and re-emit shapes. "
                          "Press 0 or 1 for remaining emitted shapes to be disks or squares.");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine, "Press '+' or '-' to speed up or slow down rotation.");
        m_textLine += DRAW_STRING_NEW_LINE;

        if (m_count < Count)
        {
            const auto body = m_world->CreateBody(BodyDef{}
                                                  .UseType(BodyType::Dynamic)
                                                  .UseLocation(Vec2(0, 10) * Meter)
                                                  .UseUserData(reinterpret_cast<void*>(1)));
            switch (m_shapeType)
            {
                case ShapeType::Square:
                    body->CreateFixture(m_square);
                    break;
                case ShapeType::Disk:
                    body->CreateFixture(m_disk);
                    break;
            }
            ++m_count;
        }
    }

    void KeyboardDown(Key key) override
    {
        const auto selectedFixture = GetSelectedFixture();
        const auto selectedShape = selectedFixture?
            selectedFixture->GetShape().get(): static_cast<Shape*>(nullptr);

        switch (key)
        {
            case Key_Add:
                if (selectedShape)
                    selectedShape->GetDensity();
                else
                    m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() + 0.01f * Pi * RadianPerSecond);
                break;
            
            case Key_Subtract:
                m_joint->SetMotorSpeed(m_joint->GetMotorSpeed() - 0.01f * Pi * RadianPerSecond);
                break;
                
            case Key_0:
                m_shapeType = ShapeType::Disk;
                break;
                
            case Key_1:
                m_shapeType = ShapeType::Square;
                break;
                
            case Key_C:
            {
                std::vector<Body*> bodies;
                for (auto&& body: m_world->GetBodies())
                {
                    auto& b = GetRef(body);
                    if (b.GetUserData() == reinterpret_cast<void*>(1))
                    {
                        bodies.push_back(&b);
                    }
                }
                for (auto&& b: bodies)
                {
                    m_world->Destroy(b);
                }
                m_count = 0;
                break;
            }

            default:
                break;
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
