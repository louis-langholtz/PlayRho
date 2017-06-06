/*
* Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef CHARACTER_COLLISION_H
#define CHARACTER_COLLISION_H

#include "../Framework/Test.hpp"

namespace box2d {

/// This is a test of typical character collision scenarios. This does not
/// show how you should implement a character in your application.
/// Instead this is used to test smooth collision on edge chains.
class CharacterCollision : public Test
{
public:
    CharacterCollision()
    {
        // Ground body
        {
            const auto ground = m_world->CreateBody();
            ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-20.0f, 0.0f) * Meter, Vec2(20.0f, 0.0f) * Meter));
        }

        {
            BodyDef bd;
            const auto ground = m_world->CreateBody(bd);
            
            PolygonShape shape;
            
            shape.SetVertexRadius(RealNum(0.006f) * Meter);
            SetAsBox(shape, RealNum(0.5) * Meter, RealNum(0.5) * Meter, Vec2(-20.01f, 0.545f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, RealNum(0.5) * Meter, RealNum(0.5) * Meter, Vec2(-20.01f, 1.545f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, RealNum(0.5) * Meter, RealNum(0.5) * Meter, Vec2(-20.01f, 2.545f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));

            shape.SetVertexRadius(RealNum(0.006f) * Meter);
            SetAsBox(shape, RealNum(0.5) * Meter, RealNum(0.5) * Meter, Vec2(-17.99f, 0.545f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, RealNum(0.5) * Meter, RealNum(0.5) * Meter, Vec2(-17.99f, 1.545f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, RealNum(0.5) * Meter, RealNum(0.5) * Meter, Vec2(-17.99f, 2.545f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        // Collinear edges with no adjacency information.
        // This shows the problematic case where a box shape can hit
        // an internal vertex.
        {
            const auto ground = m_world->CreateBody();

            EdgeShape shape;
            shape.Set(Vec2(-8.0f, 1.0f) * Meter, Vec2(-6.0f, 1.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
            shape.Set(Vec2(-6.0f, 1.0f) * Meter, Vec2(-4.0f, 1.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
            shape.Set(Vec2(-4.0f, 1.0f) * Meter, Vec2(-2.0f, 1.0f) * Meter);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
        }

        // Collinear 2-gons with no adjacency information.
        {
            const auto ground = m_world->CreateBody();
            
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
            BodyDef bd;
            bd.angle = RealNum{0.25f} * Radian * Pi;
            const auto ground = m_world->CreateBody(bd);

            Length2D vs[4];
            vs[0] = Vec2(5.0f, 7.0f) * Meter;
            vs[1] = Vec2(6.0f, 8.0f) * Meter;
            vs[2] = Vec2(7.0f, 8.0f) * Meter;
            vs[3] = Vec2(8.0f, 7.0f) * Meter;
            auto shape = std::make_shared<ChainShape>();
            shape->CreateChain(Span<const Length2D>(vs, 4));
            ground->CreateFixture(shape);
        }

        // Square tiles. This shows that adjacency shapes may
        // have non-smooth collision. There is no solution
        // to this problem.
        {
            BodyDef bd;
            const auto ground = m_world->CreateBody(bd);

            PolygonShape shape;
            SetAsBox(shape, RealNum{1.0f} * Meter, RealNum{1.0f} * Meter, Vec2(4.0f, 3.0f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, RealNum{1.0f} * Meter, RealNum{1.0f} * Meter, Vec2(6.0f, 3.0f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
            SetAsBox(shape, RealNum{1.0f} * Meter, RealNum{1.0f} * Meter, Vec2(8.0f, 3.0f) * Meter, RealNum{0.0f} * Radian);
            ground->CreateFixture(std::make_shared<PolygonShape>(shape));
        }

        // Square made from an edge loop. Collision should be smooth.
        {
            BodyDef bd;
            const auto ground = m_world->CreateBody(bd);

            Length2D vs[4];
            vs[0] = Vec2(-1.0f, 3.0f) * Meter;
            vs[1] = Vec2(1.0f, 3.0f) * Meter;
            vs[2] = Vec2(1.0f, 5.0f) * Meter;
            vs[3] = Vec2(-1.0f, 5.0f) * Meter;
            auto shape = std::make_shared<ChainShape>();
            shape->CreateLoop(Span<const Length2D>(vs, 4));
            ground->CreateFixture(shape);
        }

        // Edge loop. Collision should be smooth.
        {
            BodyDef bd;
            bd.position = Vec2(-10.0f, 4.0f) * Meter;
            const auto ground = m_world->CreateBody(bd);

            Length2D vs[10];
            vs[0] = Vec2(0.0f, 0.0f) * Meter;
            vs[1] = Vec2(6.0f, 0.0f) * Meter;
            vs[2] = Vec2(6.0f, 2.0f) * Meter;
            vs[3] = Vec2(4.0f, 1.0f) * Meter;
            vs[4] = Vec2(2.0f, 2.0f) * Meter;
            vs[5] = Vec2(0.0f, 2.0f) * Meter;
            vs[6] = Vec2(-2.0f, 2.0f) * Meter;
            vs[7] = Vec2(-4.0f, 3.0f) * Meter;
            vs[8] = Vec2(-6.0f, 2.0f) * Meter;
            vs[9] = Vec2(-6.0f, 0.0f) * Meter;
            auto shape = std::make_shared<ChainShape>();
            shape->CreateLoop(Span<const Length2D>(vs, 10));
            ground->CreateFixture(shape);
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
            conf.friction = RealNum(0);
            conf.density = RealNum{20} * KilogramPerSquareMeter;
            conf.vertexRadius = RealNum(0.006f) * Meter;
            const auto square = std::make_shared<PolygonShape>(RealNum{0.5f} * Meter, RealNum{0.5f} * Meter, conf);
            body->CreateFixture(square);
            
            bd.position = Vec2(-19.5f, 1.0f) * Meter;
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
            conf.density = RealNum{20} * KilogramPerSquareMeter;
            body->CreateFixture(std::make_shared<PolygonShape>(RealNum{0.25f} * Meter, RealNum{0.25f} * Meter, conf));
        }

        // Hexagon character
        {
            BodyDef bd;
            bd.position = Vec2(-5.0f, 8.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            const auto body = m_world->CreateBody(bd);

            auto angle = RealNum{0.0f};
            const auto delta = RealNum{Pi / 3.0f};
            Length2D vertices[6];
            for (auto i = 0; i < 6; ++i)
            {
                vertices[i] = Vec2(0.5f * std::cos(angle), 0.5f * std::sin(angle)) * Meter;
                angle += delta;
            }

            auto conf = PolygonShape::Conf{};
            conf.density = RealNum{20} * KilogramPerSquareMeter;
            auto hexshape = std::make_shared<PolygonShape>(conf);
            hexshape->Set(Span<const Length2D>(vertices, 6));
            body->CreateFixture(hexshape);
        }

        // Circle character
        {
            BodyDef bd;
            bd.position = Vec2(3.0f, 5.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.fixedRotation = true;
            bd.allowSleep = false;

            const auto body = m_world->CreateBody(bd);
            auto conf = CircleShape::Conf{};
            conf.density = RealNum{20} * KilogramPerSquareMeter;
            conf.vertexRadius = RealNum{0.5f} * Meter;
            body->CreateFixture(std::make_shared<CircleShape>(conf));
        }

        // Circle character
        {
            BodyDef bd;
            bd.position = Vec2(-7.0f, 6.0f) * Meter;
            bd.type = BodyType::Dynamic;
            bd.allowSleep = false;

            m_character = m_world->CreateBody(bd);

            auto conf = CircleShape::Conf{};
            conf.density = RealNum{20} * KilogramPerSquareMeter;
            conf.friction = 1.0f;
            conf.vertexRadius = RealNum{0.25f} * Meter;
            m_character->CreateFixture(std::make_shared<CircleShape>(conf));
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        auto velocity = m_character->GetVelocity();
        velocity.linear.x = RealNum{-5.0f} * MeterPerSecond;
        m_character->SetVelocity(velocity);
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine, "This tests various character collision shapes.");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine, "Limitation: square and hexagon can snag on aligned boxes.");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine, "Feature: edge chains have smooth collision inside and out.");
        m_textLine += DRAW_STRING_NEW_LINE;
    }

    Body* m_character;
};

} // namespace box2d

#endif
