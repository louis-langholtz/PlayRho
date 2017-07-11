/*
 * Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#ifndef AddPair_H
#define AddPair_H

#include "../Framework/Test.hpp"

namespace box2d {

class AddPair : public Test
{
public:

    AddPair()
    {
        m_world->SetGravity(LinearAcceleration2D{Real(0) * MeterPerSquareSecond, Real(0) * MeterPerSquareSecond});
        {
            auto conf = DiskShape::Conf{};
            conf.vertexRadius = Meter / Real{10};
            conf.density = Real{0.01f} * KilogramPerSquareMeter;
            const auto shape = std::make_shared<DiskShape>(conf);

            const auto minX = -6.0f;
            const auto maxX = 0.0f;
            const auto minY = 4.0f;
            const auto maxY = 6.0f;

            for (auto i = 0; i < 400; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.position = Vec2(RandomFloat(minX, maxX), RandomFloat(minY, maxY)) * Meter;
                const auto body = m_world->CreateBody(bd);
                body->CreateFixture(shape);
            }
        }

        {
            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.position = Length2D{Real(-40.0f) * Meter, Real(5.0f) * Meter};
            bd.bullet = true;
            const auto body = m_world->CreateBody(bd);
            auto conf = PolygonShape::Conf{};
            conf.density = Real{1.0f} * KilogramPerSquareMeter;
            body->CreateFixture(std::make_shared<PolygonShape>(Real{1.5f} * Meter, Real{1.5f} * Meter, conf));
            body->SetVelocity(Velocity{Vec2(150.0f, 0.0f) * MeterPerSecond, AngularVelocity{0}});
        }
    }
};

} // namespace box2d

#endif
