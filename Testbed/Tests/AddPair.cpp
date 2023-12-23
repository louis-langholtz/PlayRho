/*
 * Original work Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

#include "../Framework/Test.hpp"

namespace testbed {

class AddPair : public Test
{
public:
    static inline const auto registered =
        RegisterTest("Add Pair Stress Test", MakeUniqueTest<AddPair>);

    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "Stresses the physics engine's contact detecting and adding code.";
        return conf;
    }

    AddPair() : Test(GetTestConf())
    {
        SetGravity(LinearAcceleration2{});
        {
            const auto minX = -6.0f;
            const auto maxX = 0.0f;
            const auto minY = 4.0f;
            const auto maxY = 6.0f;
            const auto bd = BodyConf{}.Use(BodyType::Dynamic);
            const auto shape =
                CreateShape(GetWorld(), DiskShapeConf{}.UseRadius(0.1_m).UseDensity(0.01_kgpm2));
            for (auto i = 0; i < 400; ++i) {
                const auto location =
                    Vec2(RandomFloat(minX, maxX), RandomFloat(minY, maxY)) * Meter;
                // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
                Attach(GetWorld(), CreateBody(GetWorld(), BodyConf(bd).UseLocation(location)),
                       shape);
            }
        }
        const auto bd = BodyConf{}
                            .Use(BodyType::Dynamic)
                            .UseBullet(true)
                            .UseLocation(Length2{-40_m, 5_m})
                            .UseLinearVelocity(LinearVelocity2{150_mps, 0_mps});
        Attach(
            GetWorld(), CreateBody(GetWorld(), bd),
            CreateShape(GetWorld(), PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(1.5_m, 1.5_m)));
    }
};

} // namespace testbed
