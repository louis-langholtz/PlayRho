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

#ifndef EDGE_TEST_H
#define EDGE_TEST_H

#include "../Framework/Test.hpp"

namespace box2d {

class EdgeTest : public Test
{
public:

    EdgeTest()
    {
        {
            const auto ground = m_world->CreateBody();

            const auto v1 = Vec2(-10.0f, 0.0f) * Meter;
            const auto v2 = Vec2(-7.0f, -2.0f) * Meter;
            const auto v3 = Vec2(-4.0f, 0.0f) * Meter;
            const auto v4 = Vec2(0.0f, 0.0f) * Meter;
            const auto v5 = Vec2(4.0f, 0.0f) * Meter;
            const auto v6 = Vec2(7.0f, 2.0f) * Meter;
            const auto v7 = Vec2(10.0f, 0.0f) * Meter;

            EdgeShape shape;

            shape.Set(v1, v2);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            shape.Set(v2, v3);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            shape.Set(v3, v4);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            shape.Set(v4, v5);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            shape.Set(v5, v6);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));

            shape.Set(v6, v7);
            ground->CreateFixture(std::make_shared<EdgeShape>(shape));
        }

        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(-0.5f, 0.6f) * Meter;
            bd.allowSleep = false;
            const auto body = m_world->CreateBody(bd);

            auto conf = DiskShape::Conf{};
            conf.density = RealNum{1} * KilogramPerSquareMeter;
            conf.vertexRadius = RealNum{0.5f} * Meter;
            body->CreateFixture(std::make_shared<DiskShape>(conf));
        }

        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Vec2(1.0f, 0.6f) * Meter;
            bd.allowSleep = false;
            const auto body = m_world->CreateBody(bd);

            auto shape = PolygonShape{};
            shape.SetVertexRadius(RealNum{1} * Meter);
            shape.SetAsBox(RealNum{0.5f} * Meter, RealNum{0.5f} * Meter);
            shape.SetDensity(RealNum{1} * KilogramPerSquareMeter);
            body->CreateFixture(std::make_shared<PolygonShape>(shape));
        }
    }
};

} // namespace box2d

#endif
