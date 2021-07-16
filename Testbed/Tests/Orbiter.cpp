/*
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "../Framework/Test.hpp"

namespace testbed {

class Orbiter : public Test
{
public:
    static inline const auto registered = RegisterTest("Encircled Orbiter", MakeUniqueTest<Orbiter>);

    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.settings.drawSkins = true;
        conf.neededSettings = (0x1u << NeedDrawSkinsField);
        conf.description =
            "Demonstrates an enclosing looped chain shape attached to a dynamic "
            "body and gravitationally effected by another dynamic body that's orbiting a "
            "central static body. Wait a few seconds for this simulation to really get going.";
        return conf;
    }

    Orbiter() : Test(GetTestConf())
    {
        SetGravity(LinearAcceleration2{});
        {
            auto bd = BodyConf{};
            bd.type = BodyType::Static;
            bd.location = m_center;
            bd.shape = CreateShape(GetWorld(), DiskShapeConf{}.UseRadius(3_m));
            CreateBody(GetWorld(), bd);
        }
        {
            const auto radius = Real{12.0f};
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.location = Length2{GetX(m_center), GetY(m_center) + radius * 1_m};
            bd.shape =
                CreateShape(GetWorld(), DiskShapeConf{}.UseRadius(0.5_m).UseDensity(1e12_kgpm2));
            bd.linearVelocity = Vec2{Pi * radius / 2, 0} * 1_mps;
            bd.angularVelocity = 360_deg / 1_s;
            m_orbiter = CreateBody(GetWorld(), bd);
        }
        {
            auto conf = ChainShapeConf{};
            conf.Set(GetCircleVertices(16_m, 180));
            conf.UseVertexRadius(0.1_m);
            conf.UseDensity(1e3_kgpm2);
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.location = m_center;
            bd.bullet = true;
            bd.shape = CreateShape(GetWorld(), conf);
            CreateBody(GetWorld(), bd);
        }
    }

    void PreStep(const Settings&, Drawer&) override
    {
        SetAccelerations(GetWorld(), CalcGravitationalAcceleration);
        const auto force = GetCentripetalForce(GetWorld(), m_orbiter, m_center);
        const auto linAccel = force * GetInvMass(GetWorld(), m_orbiter);
        const auto angAccel = 0 * RadianPerSquareSecond;
        SetAcceleration(GetWorld(), m_orbiter, linAccel, angAccel);
    }

private:
    BodyID m_orbiter = InvalidBodyID;
    Length2 const m_center = Vec2{0, 20} * 1_m;
};

} // namespace testbed
