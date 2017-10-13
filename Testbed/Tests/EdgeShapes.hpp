/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_EDGE_SHAPES_HPP
#define  PLAYRHO_EDGE_SHAPES_HPP

#include "../Framework/Test.hpp"
#include <cmath>
#include <cstring>

namespace playrho {

class EdgeShapes : public Test
{
public:

    enum
    {
        e_maxBodies = 256
    };

    EdgeShapes()
    {
        m_circle->SetFriction(Real(0.3f));
        m_circle->SetDensity(Real{20} * KilogramPerSquareMeter);

        // Ground body
        {
            const auto ground = m_world->CreateBody();
            auto x1 = -20.0f;
            auto y1 = 2.0f * std::cos(x1 / 10.0f * static_cast<float>(Pi));
            for (auto i = 0; i < 80; ++i)
            {
                const auto x2 = x1 + 0.5f;
                const auto y2 = 2.0f * std::cos(x2 / 10.0f * static_cast<float>(Pi));
                ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(x1, y1) * Meter, Vec2(x2, y2) * Meter));
                x1 = x2;
                y1 = y2;
            }
        }

        for (auto i = 0; i < 4; ++i)
        {
            m_polygons[i] = std::make_shared<PolygonShape>();
            m_polygons[i]->SetFriction(Real(0.3f));
            m_polygons[i]->SetDensity(Real{20} * KilogramPerSquareMeter);
        }
        
        m_polygons[0]->Set({
            Vec2(-0.5f, 0.0f) * Meter,
            Vec2(0.5f, 0.0f) * Meter,
            Vec2(0.0f, 1.5f) * Meter
        });
        m_polygons[1]->Set({
            Vec2(-0.1f, 0.0f) * Meter,
            Vec2(0.1f, 0.0f) * Meter,
            Vec2(0.0f, 1.5f) * Meter
        });

        {
            const auto w = 1.0f;
            const auto b = w / (2.0f + Sqrt(2.0f));
            const auto s = Sqrt(2.0f) * b;

            m_polygons[2]->Set({
                Vec2(0.5f * s, 0.0f) * Meter,
                Vec2(0.5f * w, b) * Meter,
                Vec2(0.5f * w, b + s) * Meter,
                Vec2(0.5f * s, w) * Meter,
                Vec2(-0.5f * s, w) * Meter,
                Vec2(-0.5f * w, b + s) * Meter,
                Vec2(-0.5f * w, b) * Meter,
                Vec2(-0.5f * s, 0.0f) * Meter
            });
        }

        m_polygons[3]->SetAsBox(Real{0.5f} * Meter, Real{0.5f} * Meter);

        m_bodyIndex = 0;
        std::memset(m_bodies, 0, sizeof(m_bodies));

        m_angle = 0.0f;
    }

    void Create(int index)
    {
        if (m_bodies[m_bodyIndex])
        {
            m_world->Destroy(m_bodies[m_bodyIndex]);
            m_bodies[m_bodyIndex] = nullptr;
        }

        BodyDef bd;

        const auto x = RandomFloat(-10.0f, 10.0f);
        const auto y = RandomFloat(10.0f, 20.0f);
        bd.location = Vec2(x, y) * Meter;
        bd.angle = Radian * RandomFloat(-Pi, Pi);
        bd.type = BodyType::Dynamic;

        if (index == 4)
        {
            bd.angularDamping = Real(0.02f) * Hertz;
        }

        m_bodies[m_bodyIndex] = m_world->CreateBody(bd);

        if (index < 4)
        {
            m_bodies[m_bodyIndex]->CreateFixture(m_polygons[index]);
        }
        else
        {
            m_bodies[m_bodyIndex]->CreateFixture(m_circle);
        }

        m_bodyIndex = GetModuloNext(m_bodyIndex, static_cast<decltype(m_bodyIndex)>(e_maxBodies));
    }

    void Destroy()
    {
        for (auto i = 0; i < e_maxBodies; ++i)
        {
            if (m_bodies[i])
            {
                m_world->Destroy(m_bodies[i]);
                m_bodies[i] = nullptr;
                return;
            }
        }
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_1:
        case Key_2:
        case Key_3:
        case Key_4:
        case Key_5:
            Create(key - Key_1);
            break;

        case Key_D:
            Destroy();
            break;
                
        default:
            break;
        }
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, Drawer::Left, "Press 1-5 to drop stuff");
        m_textLine += DRAW_STRING_NEW_LINE;

        const auto L = Real(25);
        const auto point1 = Vec2(0.0f, 10.0f) * Meter;
        const auto d = Vec2(L * std::cos(m_angle), -L * Abs(std::sin(m_angle))) * Meter;
        const auto point2 = point1 + d;

        auto fixture = static_cast<Fixture*>(nullptr);
        Length2D point;
        UnitVec2 normal;

        m_world->RayCast(point1, point2, [&](Fixture* f, ChildCounter, Length2D p, UnitVec2 n) {
            fixture = f;
            point = p;
            normal = n;
            return World::RayCastOpcode::ClipRay;
        });

        if (fixture)
        {
            drawer.DrawPoint(point, 5.0f, Color(0.4f, 0.9f, 0.4f));
            drawer.DrawSegment(point1, point, Color(0.8f, 0.8f, 0.8f));
            const auto head = point + Real{0.5f} * normal * Meter;
            drawer.DrawSegment(point, head, Color(0.9f, 0.9f, 0.4f));
        }
        else
        {
            drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
        }

        const auto advanceRay = !settings.pause || settings.singleStep;
        if (advanceRay)
        {
            m_angle += 0.25f * Pi / 180.0f;
        }
    }

    int m_bodyIndex;
    Body* m_bodies[e_maxBodies];
    std::shared_ptr<PolygonShape> m_polygons[4];
    std::shared_ptr<DiskShape> m_circle = std::make_shared<DiskShape>(Real{0.5f} * Meter);

    Real m_angle;
};

} // namespace playrho

#endif
