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

#ifndef CHARACTER_COLLISION_H
#define CHARACTER_COLLISION_H

#include "../Framework/Test.hpp"

namespace box2d {

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on surfaces.
class CharacterCollision : public Test
{
public:
    CharacterCollision()
    {
        // Ground body
        const auto ground = m_world->CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f) * Meter, Vec2(20.0f, 0.0f) * Meter));

        {
            PolygonShape shape;

            shape.SetVertexRadius(Real(0.006f) * Meter);
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(20.01f, 0.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(20.01f, 1.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(20.01f, 2.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(20.01f, 3.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(20.01f, 4.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(20.01f, 5.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(20.01f, 6.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));

            shape.SetVertexRadius(Real(0.006f) * Meter);
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(17.99f, 0.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(17.99f, 1.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(17.99f, 2.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(17.99f, 3.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(17.99f, 4.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(17.99f, 5.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real(0.5) * Meter, Real(0.5) * Meter,
                     Vec2(17.99f, 6.545f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        // Collinear edges.
        {
            EdgeShape shape;
            shape.Set(Vec2(-8.0f, 1.0f) * Meter, Vec2(-6.0f, 1.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
            shape.Set(Vec2(-6.0f, 1.0f) * Meter, Vec2(-4.0f, 1.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
            shape.Set(Vec2(-4.0f, 1.0f) * Meter, Vec2(-2.0f, 1.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
        }

        // Collinear 2-gons.
        {
            PolygonShape shape;
            shape.Set({Vec2(-8.0f, 20.0f) * Meter, Vec2(-6.0f, 20.0f) * Meter});
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            shape.Set({Vec2(-6.0f, 20.0f) * Meter, Vec2(-4.0f, 20.0f) * Meter});
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            shape.Set({Vec2(-4.0f, 20.0f) * Meter, Vec2(-2.0f, 20.0f) * Meter});
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        // Chain shape
        {
            const auto body = m_world->CreateBody(BodyDef{}.UseAngle(Real{45} * Degree));
            auto conf = ChainShape::Conf{};
            conf.vertices.push_back(Vec2(5.0f, 7.0f) * Meter);
            conf.vertices.push_back(Vec2(6.0f, 8.0f) * Meter);
            conf.vertices.push_back(Vec2(7.0f, 8.0f) * Meter);
            conf.vertices.push_back(Vec2(8.0f, 7.0f) * Meter);
            body->CreateFixture(std::make_shared<ChainShape>(conf));
        }

        // Square tiles.
        {
            PolygonShape shape;
            SetAsBox(shape, Real{1.0f} * Meter, Real{1.0f} * Meter, Vec2(4.0f, 3.0f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real{1.0f} * Meter, Real{1.0f} * Meter, Vec2(6.0f, 3.0f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, Real{1.0f} * Meter, Real{1.0f} * Meter, Vec2(8.0f, 3.0f) * Meter, Real{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        // Square made from an edge loop. Collision should be smooth.
        {
            auto conf = ChainShape::Conf{};
            conf.vertices.push_back(Vec2(-1.0f, 3.0f) * Meter);
            conf.vertices.push_back(Vec2(1.0f, 3.0f) * Meter);
            conf.vertices.push_back(Vec2(1.0f, 5.0f) * Meter);
            conf.vertices.push_back(Vec2(-1.0f, 5.0f) * Meter);
            conf.vertices.push_back(conf.vertices[0]); // to loop chain shape around
            ground->CreateFixture(std::make_shared<ChainShape>(conf));
        }

        // Edge loop. Collision should be smooth.
        {
            const auto body = m_world->CreateBody(BodyDef{}.UseLocation(Vec2(-10.0f, 4.0f) * Meter));
            auto conf = ChainShape::Conf{};
            conf.vertices.push_back(Vec2(0.0f, 0.0f) * Meter);
            conf.vertices.push_back(Vec2(6.0f, 0.0f) * Meter);
            conf.vertices.push_back(Vec2(6.0f, 2.0f) * Meter);
            conf.vertices.push_back(Vec2(4.0f, 1.0f) * Meter);
            conf.vertices.push_back(Vec2(2.0f, 2.0f) * Meter);
            conf.vertices.push_back(Vec2(0.0f, 2.0f) * Meter);
            conf.vertices.push_back(Vec2(-2.0f, 2.0f) * Meter);
            conf.vertices.push_back(Vec2(-4.0f, 3.0f) * Meter);
            conf.vertices.push_back(Vec2(-6.0f, 2.0f) * Meter);
            conf.vertices.push_back(Vec2(-6.0f, 0.0f) * Meter);
            conf.vertices.push_back(conf.vertices[0]); // to loop back completely.
            body->CreateFixture(std::make_shared<ChainShape>(conf));
        }

        // Square character 1
        {
            BodyDef bd;
            bd.position = Vec2(-3.0f, 8.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = false;
            bd.allowSleep = false;

            const auto body = m_world->CreateBody(bd);

            auto conf = PolygonShape::Conf{};
            conf.friction = Real(0);
            conf.density = Real{20} * KilogramPerSquareMeter;
            conf.vertexRadius = Real(0.006f) * Meter;
            const auto square = std::make_shared<PolygonShape>(Real{0.5f} * Meter, Real{0.5f} * Meter, conf);
            body->CreateFixture(square);
            
            bd.position = Vec2(19.0f, 7.0f) * Meter;
            const auto body2 = m_world->CreateBody(bd);
            body2->CreateFixture(square);
        }

        // Square character 2
        {
            BodyDef bd;
            bd.position = Vec2(-5.0f, 5.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            const auto body = m_world->CreateBody(bd);

            auto conf = PolygonShape::Conf{};
            conf.density = Real{20} * KilogramPerSquareMeter;
            body->CreateFixture(std::make_shared<PolygonShape>(Real{0.25f} * Meter, Real{0.25f} * Meter, conf));
        }

        // Hexagon character
        {
            BodyDef bd;
            bd.position = Vec2(-5.0f, 8.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            const auto body = m_world->CreateBody(bd);

            auto angle = Real{0.0f};
            const auto delta = Real{Pi / 3.0f};
            Length2D vertices[6];
            for (auto i = 0; i < 6; ++i)
            {
                vertices[i] = Vec2(0.5f * std::cos(angle), 0.5f * std::sin(angle)) * Meter;
                angle += delta;
            }

            auto conf = PolygonShape::Conf{};
            conf.density = Real{20} * KilogramPerSquareMeter;
            auto hexshape = std::make_shared<PolygonShape>(conf);
            hexshape->Set(Span<const Length2D>(vertices, 6));
            body->CreateFixture(hexshape);
        }

        // Disk character
        {
            BodyDef bd;
            bd.position = Vec2(3.0f, 5.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            const auto body = m_world->CreateBody(bd);
            auto conf = DiskShape::Conf{};
            conf.density = Real{20} * KilogramPerSquareMeter;
            conf.vertexRadius = Real{0.5f} * Meter;
            body->CreateFixture(std::make_shared<DiskShape>(conf));
        }

        // Disk character
        {
            BodyDef bd;
            bd.position = Vec2(-7.0f, 6.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.allowSleep = false;

            m_character = m_world->CreateBody(bd);

            auto conf = DiskShape::Conf{};
            conf.density = Real{20} * KilogramPerSquareMeter;
            conf.friction = 1.0f;
            conf.vertexRadius = Real{0.25f} * Meter;
            m_character->CreateFixture(std::make_shared<DiskShape>(conf));
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        auto velocity = m_character->GetVelocity();
        velocity.linear.x = Real{-5.0f} * MeterPerSecond;
        m_character->SetVelocity(velocity);
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, "This tests various character collision shapes for snag-free smooth sliding.");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    Body* m_character;
};

} // namespace box2d

#endif
