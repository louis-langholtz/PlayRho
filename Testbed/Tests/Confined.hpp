/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#ifndef CONFINED_H
#define CONFINED_H

#include "../Framework/Test.hpp"

namespace box2d {

class Confined : public Test
{
public:
    const Length wall_length = Real(0.15f) * Meter; // DefaultLinearSlop * 1000
    const Length vertexRadiusIncrement = wall_length / Real{40};
    
    enum
    {
        e_columnCount = 0,
        e_rowCount = 0
    };

    Confined()
    {
        m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);

        const auto radius = Real{0.5f} * Meter;
        auto conf = DiskShape::Conf{};
        conf.vertexRadius = radius;
        conf.density = Real{1} * KilogramPerSquareMeter;
        conf.friction = 0.1f;
        const auto shape = std::make_shared<DiskShape>(conf);

        for (auto j = 0; j < e_columnCount; ++j)
        {
            for (auto i = 0; i < e_rowCount; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2{
                    -10.0f + (2.1f * j + 1.0f + 0.01f * i) * (radius / Meter),
                    (2.0f * i + 1.0f) * (radius/ Meter)
                } * Meter;
                const auto body = m_world->CreateBody(bd);
                body->CreateFixture(shape);
            }
        }

        m_world->SetGravity(Vec2(0.0f, 0.0f) * MeterPerSquareSecond);
    }

    Body* CreateEnclosure(Length vertexRadius, Length wallLength)
    {
        const auto ground = m_world->CreateBody();
        
        auto conf = ChainShape::Conf{};
        conf.restitution = 0; // originally 0.9
        conf.vertexRadius = vertexRadius;
        
        const auto btmLeft = Length2D(-wallLength / Real{2}, Real{0} * Meter);
        const auto btmRight = Length2D(wallLength / Real{2}, Real{0} * Meter);
        const auto topLeft = Length2D(-wallLength / Real{2}, wallLength);
        const auto topRight = Length2D(wallLength / Real{2}, wallLength);
        
        conf.vertices.push_back(btmRight);
        conf.vertices.push_back(topRight);
        conf.vertices.push_back(topLeft);
        conf.vertices.push_back(btmLeft);
        conf.vertices.push_back(conf.vertices[0]);

        ground->CreateFixture(std::make_shared<ChainShape>(conf));
        
        return ground;
    }
    
    void CreateCircle()
    {
        const auto radius = wall_length/Real{10}; // 2

        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.bullet = m_bullet_mode;
        const auto wl = StripUnit(wall_length);
        bd.position = Vec2(RandomFloat(-wl / Real{2}, +wl / Real{2}), RandomFloat(0, wl)) * Meter;
        bd.userData = reinterpret_cast<void*>(m_sequence);
        //bd.allowSleep = false;

        const auto body = m_world->CreateBody(bd);
        
        auto conf = DiskShape::Conf{};
        conf.density = Real{1} * KilogramPerSquareMeter;
        conf.restitution = 0.8f;
        conf.vertexRadius = radius;
        body->CreateFixture(std::make_shared<DiskShape>(conf));

        ++m_sequence;
    }

    void CreateBox()
    {
        const auto side_length = wall_length / Real{5}; // 4

        auto conf = PolygonShape::Conf{};
        conf.density = Real{1} * KilogramPerSquareMeter;
        conf.restitution = 0; // originally 0.8
        
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.bullet = m_bullet_mode;
        const auto wl = StripUnit(wall_length);
        bd.position = Vec2(RandomFloat(-wl / Real{2}, +wl / Real{2}), RandomFloat(0, wl)) * Meter;
        bd.userData = reinterpret_cast<void*>(m_sequence);
        const auto body = m_world->CreateBody(bd);
        body->CreateFixture(std::make_shared<PolygonShape>(side_length/Real{2}, side_length/Real{2}, conf));

        ++m_sequence;
    }

    void ToggleBulletMode()
    {
        m_bullet_mode = !m_bullet_mode;
        for (auto&& body: m_world->GetBodies())
        {
            auto& b = GetRef(body);
            if (b.GetType() == BodyType::Dynamic)
            {
                b.SetBullet(m_bullet_mode);
            }
        }
    }

    void ImpartRandomImpulses()
    {
        for (auto&& body: m_world->GetBodies())
        {
            auto& b = GetRef(body);
            if (b.GetType() == BodyType::Dynamic)
            {
                const auto position = b.GetLocation();
                const auto centerPos = Length2D{position.x, position.y - (wall_length / Real{2})};
                const auto angle_from_center = GetAngle(centerPos);
                const auto direction = angle_from_center + Pi * Radian;
                const auto magnitude = Sqrt(Square(StripUnit(wall_length)) * Real{2}) *
                	GetMass(b) * Real{20} * MeterPerSecond;
                const auto impulse = Momentum2D{magnitude * UnitVec2{direction}};
                ApplyLinearImpulse(b, impulse, b.GetWorldCenter());
            }
        }        
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_C:
            CreateCircle();
            break;
        case Key_B:
            CreateBox();
            break;
        case Key_I:
            ImpartRandomImpulses();
            break;
        case Key_Period:
            ToggleBulletMode();
            break;
        case Key_Add:
            m_world->Destroy(m_enclosure);
            m_enclosureVertexRadius += vertexRadiusIncrement;
            m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
            break;
        case Key_Subtract:
            m_world->Destroy(m_enclosure);
            m_enclosureVertexRadius -= vertexRadiusIncrement;
            if (m_enclosureVertexRadius < Length{0})
            {
                m_enclosureVertexRadius = 0;
            }
            m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
            break;
        default:
            break;
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        auto sleeping = true;
        for (auto&& body: m_world->GetBodies())
        {
            auto& b = GetRef(body);

            if (b.GetType() != BodyType::Dynamic)
            {
                continue;
            }

            if (b.IsAwake())
            {
                sleeping = false;
            }
        }

        //if (sleeping)
        //{
        //    CreateCircle();
        //}
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        for (auto& body: m_world->GetBodies())
        {
            auto& b = GetRef(body);
            if (b.GetType() != BodyType::Dynamic)
            {
                continue;
            }
            
            const auto location = b.GetLocation();
            const auto userData = b.GetUserData();
            drawer.DrawString(location, "B%d", reinterpret_cast<decltype(m_sequence)>(userData));
        }
        
        drawer.DrawString(5, m_textLine, "Press 'c' to create a circle.");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine, "Press 'b' to create a box.");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine, "Press '.' to toggle bullet mode (currently %s).", m_bullet_mode? "on": "off");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine, "Press 'i' to impart impulses.");
        m_textLine += DRAW_STRING_NEW_LINE;
    }
    
    bool m_bullet_mode = false;
    Length m_enclosureVertexRadius = vertexRadiusIncrement;
    Body* m_enclosure = nullptr;
    std::size_t m_sequence = 0;
};

} // namespace box2d

#endif
