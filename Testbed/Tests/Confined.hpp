/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_CONFINED_HPP
#define  PLAYRHO_CONFINED_HPP

#include "../Framework/Test.hpp"
#include <sstream>

namespace playrho {

class Confined : public Test
{
public:
    const Length wall_length = DefaultLinearSlop * 80;
    const Length vertexRadiusIncrement = wall_length / Real{40};
    
    enum
    {
        e_columnCount = 0,
        e_rowCount = 0
    };

    Confined()
    {
        m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);

        RegisterForKey(GLFW_KEY_C, GLFW_PRESS, 0, "Create Circle", [&](KeyActionMods) {
            CreateCircle();
        });
        RegisterForKey(GLFW_KEY_B, GLFW_PRESS, 0, "Create Box", [&](KeyActionMods) {
            CreateBox();
        });
        RegisterForKey(GLFW_KEY_I, GLFW_PRESS, 0, "Impart Impulse", [&](KeyActionMods) {
            ImpartRandomImpulses();
        });
        RegisterForKey(GLFW_KEY_PERIOD, GLFW_PRESS, 0, "Toggle Bullet Mode", [&](KeyActionMods) {
            ToggleBulletMode();
        });
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "Thicken The Walls", [&](KeyActionMods) {
            m_world.Destroy(m_enclosure);
            m_enclosureVertexRadius += vertexRadiusIncrement;
            m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
        });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "Thin The Walls", [&](KeyActionMods) {
            m_world.Destroy(m_enclosure);
            m_enclosureVertexRadius -= vertexRadiusIncrement;
            if (m_enclosureVertexRadius < 0_m)
            {
                m_enclosureVertexRadius = 0_m;
            }
            m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
        });
        
        const auto radius = 0.5_m;
        auto conf = DiskShape::Conf{};
        conf.vertexRadius = radius;
        conf.density = 1_kgpm2;
        conf.friction = 0.1f;
        const auto shape = std::make_shared<DiskShape>(conf);

        for (auto j = 0; j < e_columnCount; ++j)
        {
            for (auto i = 0; i < e_rowCount; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2{
                    -10.0f + (2.1f * j + 1.0f + 0.01f * i) * (radius / 1_m),
                    (2.0f * i + 1.0f) * (radius/ 1_m)
                } * 1_m;
                const auto body = m_world.CreateBody(bd);
                body->CreateFixture(shape);
            }
        }

        m_world.SetGravity(LinearAcceleration2{});
    }
    
    Body* CreateEnclosure(Length vertexRadius, Length wallLength)
    {
        const auto body = CreateSquareEnclosingBody(m_world, wallLength, ShapeConf{
            }.UseVertexRadius(vertexRadius).UseRestitution(Finite<Real>(0)));
        SetLocation(*body, Length2{0_m, 20_m});
        return body;
    }

    Length2 GetRandomOffset() const
    {
        const auto halfWL = StripUnit(wall_length) / 2;
        return Vec2{RandomFloat(-halfWL, +halfWL), RandomFloat(-halfWL, +halfWL)} * 1_m;
    }
    
    void CreateCircle()
    {
        const auto radius = wall_length/ 10; // 2

        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.bullet = m_bullet_mode;
        bd.location = Vec2{0, 20} * 1_m + GetRandomOffset();
        //bd.allowSleep = false;

        const auto body = m_world.CreateBody(bd);
        
        auto conf = DiskShape::Conf{};
        conf.density = 1_kgpm2;
        conf.restitution = 0.8f;
        conf.vertexRadius = radius;
        body->CreateFixture(std::make_shared<DiskShape>(conf));
    }

    void CreateBox()
    {
        const auto side_length = wall_length / Real{5}; // 4

        auto conf = PolygonShape::Conf{};
        conf.density = 1_kgpm2;
        conf.restitution = 0; // originally 0.8
        
        BodyDef bd;
        bd.type = BodyType::Dynamic;
        bd.bullet = m_bullet_mode;
        bd.location = Vec2{0, 20} * 1_m + GetRandomOffset();
        const auto body = m_world.CreateBody(bd);
        body->CreateFixture(std::make_shared<PolygonShape>(side_length/Real{2}, side_length/Real{2}, conf));
    }

    void ToggleBulletMode()
    {
        m_bullet_mode = !m_bullet_mode;
        for (auto&& body: m_world.GetBodies())
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
        for (auto&& body: m_world.GetBodies())
        {
            auto& b = GetRef(body);
            if (b.GetType() == BodyType::Dynamic)
            {
                const auto position = b.GetLocation();
                const auto centerPos = Length2{
                    GetX(position), GetY(position) - (wall_length / Real{2})
                };
                const auto angle_from_center = GetAngle(centerPos);
                const auto direction = angle_from_center + Pi * 1_rad;
                const auto magnitude = Sqrt(Square(StripUnit(wall_length)) * 2) *
                	GetMass(b) * 20_mps;
                const auto impulse = Momentum2{magnitude * UnitVec2::Get(direction)};
                ApplyLinearImpulse(b, impulse, b.GetWorldCenter());
            }
        }        
    }

    void PreStep(const Settings&, Drawer&) override
    {
        auto sleeping = true;
        for (auto&& body: m_world.GetBodies())
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

    void PostStep(const Settings&, Drawer&) override
    {
        std::stringstream stream;
        stream << "Bullet mode currently " << (m_bullet_mode? "on": "off") << ".";
        m_status = stream.str();
    }
    
    bool m_bullet_mode = false;
    Length m_enclosureVertexRadius = vertexRadiusIncrement;
    Body* m_enclosure = nullptr;
};

} // namespace playrho

#endif
