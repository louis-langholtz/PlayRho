/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
* Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_CHARACTER_COLLISION_HPP
#define  PLAYRHO_CHARACTER_COLLISION_HPP

#include "../Framework/Test.hpp"

namespace testbed {

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on surfaces.
class CharacterCollision : public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "Tests various character shapes for snag-free smooth sliding.";
        return conf;
    }

    CharacterCollision(): Test(GetTestConf())
    {
        // Ground body
        const auto ground = CreateBody(GetWorld());
        Attach(GetWorld(), ground, CreateShape(GetWorld(), EdgeShapeConf{Vec2(-20, 0) * 1_m, Vec2(20, 0) * 1_m}));

        {
            auto shape = PolygonShapeConf{};

            shape.SetAsBox(0.5_m, 0.5_m, Vec2(20.015f, 0.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(20.015f, 1.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(20.015f, 2.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(20.015f, 3.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(20.015f, 4.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(20.015f, 5.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(20.015f, 6.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));

            shape.SetAsBox(0.5_m, 0.5_m, Vec2(17.985f, 0.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(17.985f, 1.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(17.985f, 2.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(17.985f, 3.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(17.985f, 4.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(17.985f, 5.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(0.5_m, 0.5_m, Vec2(17.985f, 6.545f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
        }

        // Collinear edges.
        {
            auto conf = EdgeShapeConf{};
            conf.Set(Vec2(-8.0f, 1.0f) * 1_m, Vec2(-6.0f, 1.0f) * 1_m);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.Set(Vec2(-6.0f, 1.0f) * 1_m, Vec2(-4.0f, 1.0f) * 1_m);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.Set(Vec2(-4.0f, 1.0f) * 1_m, Vec2(-2.0f, 1.0f) * 1_m);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
        }

        // Collinear 2-gons.
        {
            auto conf = PolygonShapeConf{};
            conf.UseVertices({Vec2(-8.0f, 20.0f) * 1_m, Vec2(-6.0f, 20.0f) * 1_m});
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.UseVertices({Vec2(-6.0f, 20.0f) * 1_m, Vec2(-4.0f, 20.0f) * 1_m});
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
            conf.UseVertices({Vec2(-4.0f, 20.0f) * 1_m, Vec2(-2.0f, 20.0f) * 1_m});
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
        }

        // Chain shape
        {
            const auto body = CreateBody(GetWorld(), BodyConf{}.UseAngle(45_deg));
            auto conf = ChainShapeConf{};
            conf.Add(Vec2(5.0f, 7.0f) * 1_m);
            conf.Add(Vec2(6.0f, 8.0f) * 1_m);
            conf.Add(Vec2(7.0f, 8.0f) * 1_m);
            conf.Add(Vec2(8.0f, 7.0f) * 1_m);
            Attach(GetWorld(), body, CreateShape(GetWorld(), conf));
        }

        // Square tiles.
        {
            auto shape = PolygonShapeConf{};
            shape.SetAsBox(1_m, 1_m, Vec2(4.0f, 3.0f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(1_m, 1_m, Vec2(6.0f, 3.0f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
            shape.SetAsBox(1_m, 1_m, Vec2(8.0f, 3.0f) * 1_m, 0_rad);
            Attach(GetWorld(), ground, CreateShape(GetWorld(), shape));
        }

        // Square made from an edge loop. Collision should be smooth.
        {
            auto conf = ChainShapeConf{};
            conf.Add(Vec2(-1.0f, 3.0f) * 1_m);
            conf.Add(Vec2(1.0f, 3.0f) * 1_m);
            conf.Add(Vec2(1.0f, 5.0f) * 1_m);
            conf.Add(Vec2(-1.0f, 5.0f) * 1_m);
            conf.Add(conf.GetVertex(0)); // to loop chain shape around
            Attach(GetWorld(), ground, CreateShape(GetWorld(), conf));
        }

        // Edge loop. Collision should be smooth.
        {
            const auto body = CreateBody(GetWorld(), BodyConf{}.UseLocation(Vec2(-10.0f, 4.0f) * 1_m));
            auto conf = ChainShapeConf{};
            conf.Add(Length2{});
            conf.Add(Vec2(6.0f, 0.0f) * 1_m);
            conf.Add(Vec2(6.0f, 2.0f) * 1_m);
            conf.Add(Vec2(4.0f, 1.0f) * 1_m);
            conf.Add(Vec2(2.0f, 2.0f) * 1_m);
            conf.Add(Vec2(0.0f, 2.0f) * 1_m);
            conf.Add(Vec2(-2.0f, 2.0f) * 1_m);
            conf.Add(Vec2(-4.0f, 3.0f) * 1_m);
            conf.Add(Vec2(-6.0f, 2.0f) * 1_m);
            conf.Add(Vec2(-6.0f, 0.0f) * 1_m);
            conf.Add(conf.GetVertex(0)); // to loop back completely.
            Attach(GetWorld(), body, CreateShape(GetWorld(), conf));
        }

        // Square character 1
        {
            BodyConf bd;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = false;
            bd.allowSleep = false;
            const auto square = CreateShape(GetWorld(),
                PolygonShapeConf{}.UseFriction(0).UseDensity(20_kgpm2).SetAsBox(0.5_m, 0.5_m)
            );
            bd.location = Vec2(-3.0f, 8.0f) * 1_m;
            Attach(GetWorld(), CreateBody(GetWorld(), bd), square);
            bd.location = Vec2(19.0f, 7.0f) * 1_m;
            Attach(GetWorld(), CreateBody(GetWorld(), bd), square);
        }

        // Square character 2
        {
            BodyConf bd;
            bd.location = Vec2(-5.0f, 5.0f) * 1_m;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            Attach(GetWorld(), CreateBody(GetWorld(), bd), CreateShape(GetWorld(),
                PolygonShapeConf{}.UseDensity(20_kgpm2).SetAsBox(0.25_m, 0.25_m)
            ));
        }

        // Hexagon character
        {
            BodyConf bd;
            bd.location = Vec2(-5.0f, 8.0f) * 1_m;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            const auto body = CreateBody(GetWorld(), bd);

            auto angle = Real{0.0f};
            const auto delta = Real{Pi / 3.0f};
            auto vertices = std::vector<Length2>();
            for (auto i = 0; i < 6; ++i)
            {
                vertices.push_back(Vec2(0.5f * cos(angle), 0.5f * sin(angle)) * 1_m);
                angle += delta;
            }

            auto conf = PolygonShapeConf{}.UseDensity(20_kgpm2).UseVertices(vertices);
            Attach(GetWorld(), body, CreateShape(GetWorld(), conf));
        }

        // Disk character
        {
            BodyConf bd;
            bd.location = Vec2(3.0f, 5.0f) * 1_m;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            const auto body = CreateBody(GetWorld(), bd);
            auto conf = DiskShapeConf{};
            conf.density = 20_kgpm2;
            conf.vertexRadius = 0.5_m;
            Attach(GetWorld(), body, CreateShape(GetWorld(), conf));
        }

        // Disk character
        {
            BodyConf bd;
            bd.location = Vec2(-7.0f, 6.0f) * 1_m;
            bd.type = BodyType::Dynamic;
            bd.allowSleep = false;

            m_character = CreateBody(GetWorld(), bd);

            auto conf = DiskShapeConf{};
            conf.density = 20_kgpm2;
            conf.friction = 1.0f;
            conf.vertexRadius = 0.25_m;
            Attach(GetWorld(), m_character, CreateShape(GetWorld(), conf));
        }
        
        SetAccelerations(GetWorld(), GetGravity());
    }

    void PreStep(const Settings&, Drawer&) override
    {
        auto velocity = GetVelocity(GetWorld(), m_character);
        GetX(velocity.linear) = -5_mps;
        SetVelocity(GetWorld(), m_character, velocity);
    }

    BodyID m_character;
};

} // namespace testbed

#endif
