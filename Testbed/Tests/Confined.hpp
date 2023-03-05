/*
 * Original work Copyright (c) 2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef PLAYRHO_CONFINED_HPP
#define  PLAYRHO_CONFINED_HPP

#include "../Framework/Test.hpp"
#include <sstream>

namespace testbed {

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
        SetGravity(LinearAcceleration2{});
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
            Destroy(GetWorld(), m_enclosure);
            m_enclosureVertexRadius += vertexRadiusIncrement;
            m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
        });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "Thin The Walls", [&](KeyActionMods) {
            Destroy(GetWorld(), m_enclosure);
            m_enclosureVertexRadius -= vertexRadiusIncrement;
            if (m_enclosureVertexRadius < 0_m)
            {
                m_enclosureVertexRadius = 0_m;
            }
            m_enclosure = CreateEnclosure(m_enclosureVertexRadius, wall_length);
        });
        
        const auto radius = 0.5_m;
        auto conf = DiskShapeConf{};
        conf.vertexRadius = radius;
        conf.density = 1_kgpm2;
        conf.friction = 0.1f;
        const auto shape = Shape(conf);

        for (auto j = 0; j < e_columnCount; ++j)
        {
            for (auto i = 0; i < e_rowCount; ++i)
            {
                BodyConf bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2{
                    -10.0f + (2.1f * j + 1.0f + 0.01f * i) * (radius / 1_m),
                    (2.0f * i + 1.0f) * (radius/ 1_m)
                } * 1_m;
                const auto body = CreateBody(GetWorld(), bd);
                CreateFixture(GetWorld(), body, shape);
            }
        }
    }
    
    BodyID CreateEnclosure(Length vertexRadius, Length wallLength)
    {
        const auto body = CreateBody(GetWorld());
        CreateFixture(GetWorld(), body, Shape{GetChainShapeConf(wallLength)
            .UseRestitution(0).UseVertexRadius(vertexRadius)});
        SetLocation(GetWorld(), body, Length2{0_m, 20_m});
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

        BodyConf bd;
        bd.type = BodyType::Dynamic;
        bd.bullet = m_bullet_mode;
        bd.location = Vec2{0, 20} * 1_m + GetRandomOffset();
        //bd.allowSleep = false;
        
        auto conf = DiskShapeConf{};
        conf.density = 1_kgpm2;
        conf.restitution = 0.8f;
        conf.vertexRadius = radius;
        CreateFixture(GetWorld(), CreateBody(GetWorld(), bd), Shape(conf));
    }

    void CreateBox()
    {
        const auto side_length = wall_length / Real{5}; // 4
        // originally restitution was 0.8f
        BodyConf bd;
        bd.type = BodyType::Dynamic;
        bd.bullet = m_bullet_mode;
        bd.location = Vec2{0, 20} * 1_m + GetRandomOffset();
        CreateFixture(GetWorld(), CreateBody(GetWorld(), bd), Shape{
            PolygonShapeConf{}.UseDensity(1_kgpm2).UseRestitution(0).SetAsBox(side_length/2, side_length/2)
        });
    }

    void ToggleBulletMode()
    {
        m_bullet_mode = !m_bullet_mode;
        for (const auto& b: GetBodies(GetWorld()))
        {
            if (GetType(GetWorld(), b) == BodyType::Dynamic)
            {
                if (m_bullet_mode)
                    SetImpenetrable(GetWorld(), b);
                else
                    UnsetImpenetrable(GetWorld(), b);
            }
        }
    }

    void ImpartRandomImpulses()
    {
        for (const auto& b: GetBodies(GetWorld()))
        {
            if (GetType(GetWorld(), b) == BodyType::Dynamic)
            {
                const auto position = GetLocation(GetWorld(), b);
                const auto centerPos = Length2{
                    GetX(position), GetY(position) - (wall_length / Real{2})
                };
                const auto angle_from_center = GetAngle(centerPos);
                const auto direction = angle_from_center + Pi * 1_rad;
                const auto magnitude = sqrt(Square(StripUnit(wall_length)) * 2) *
                	GetMass(GetWorld(), b) * 20_mps;
                const auto impulse = Momentum2{magnitude * UnitVec::Get(direction)};
                ApplyLinearImpulse(GetWorld(), b, impulse, GetWorldCenter(GetWorld(), b));
            }
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        auto sleeping = true;
        for (const auto& b: GetBodies(GetWorld()))
        {
            if (GetType(GetWorld(), b) != BodyType::Dynamic)
            {
                continue;
            }

            if (IsAwake(GetWorld(), b))
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
        SetStatus(stream.str());
    }
    
    bool m_bullet_mode = false;
    Length m_enclosureVertexRadius = vertexRadiusIncrement;
    BodyID m_enclosure = InvalidBodyID;
};

} // namespace testbed

#endif
