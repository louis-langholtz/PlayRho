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

#ifndef PLAYRHO_ADDPAIR_HPP
#define PLAYRHO_ADDPAIR_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class AddPair : public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "Stresses the physics engine's contact detecting and adding code.";
        return conf;
    }
    
    AddPair(): Test(GetTestConf())
    {
        m_world.SetGravity(LinearAcceleration2D{});
        {
            const auto conf = DiskShape::Conf{}
                .UseVertexRadius(Meter / 10)
                .UseDensity(0.01f * KilogramPerSquareMeter);
            const auto shape = std::make_shared<DiskShape>(conf);

            const auto minX = -6.0f;
            const auto maxX = 0.0f;
            const auto minY = 4.0f;
            const auto maxY = 6.0f;

            const auto bd = BodyDef{}.UseType(BodyType::Dynamic);
            for (auto i = 0; i < 400; ++i)
            {
                const auto location = Vec2(RandomFloat(minX, maxX), RandomFloat(minY, maxY)) * Meter;
                const auto body = m_world.CreateBody(BodyDef{bd}.UseLocation(location));
                body->CreateFixture(shape);
            }
        }

        {
            const auto bd = BodyDef{}
                .UseType(BodyType::Dynamic)
                .UseBullet(true)
                .UseLocation(Length2D{-40.0f * Meter, 5.0f * Meter})
                .UseLinearVelocity(LinearVelocity2D{Vec2(150.0f, 0.0f) * MeterPerSecond});
            const auto body = m_world.CreateBody(bd);

            const auto conf = PolygonShape::Conf{}.UseDensity(1.0f * KilogramPerSquareMeter);
            body->CreateFixture(std::make_shared<PolygonShape>(1.5f * Meter, 1.5f * Meter, conf));
        }
    }
};

} // namespace playrho

#endif
