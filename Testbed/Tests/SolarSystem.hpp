/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef SolarSystem_hpp
#define SolarSystem_hpp

#include "../Framework/Test.hpp"

namespace playrho {

/// @brief Solar system demo.
/// @sa https://en.wikipedia.org/wiki/Solar_System

struct SolarSystemObject
{
    const char* name;
    Length radius;
    Mass mass;
    Time orbitalPeriod; ///< Orbital period.
    Length aveDist; ///< Average distance.
    Time rotationalPeriod; ///< Rotational period.
};

static constexpr SolarSystemObject SolarSystemBodies[] = {
    { "Sun",    696342_km, 1988550000.0_Yg,     0.000_d,    0_Gm,   25.050_d },
    { "Mercury",  2439_km,        330.2_Yg,    87.969_d,   57_Gm,   58.646_d },
    { "Venus",    6051_km,       4868.5_Yg,   224.701_d,  108_Gm, -243.025_d },
    { "Earth",    6371_km,       5973.6_Yg,   365.256_d,  150_Gm,    0.997_d },
    { "Mars",     3389_km,        641.8_Yg,   686.971_d,  230_Gm,    1.025_d },
    { "Jupiter", 69911_km,    1898600.0_Yg,  4332.590_d,  778_Gm,    0.413_d },
    { "Saturn",  58232_km,     568460.0_Yg, 10759.220_d, 1430_Gm,    0.439_d },
    { "Uranus",  25362_km,      86832.0_Yg, 30688.500_d, 2880_Gm,   -0.718_d },
    { "Neptune", 24622_km,     102430.0_Yg, 60182.000_d, 4500_Gm,    0.671_d },
};

static constexpr auto G = 6.67408e-11f * SquareMeter * 1_m / (1_kg * Square(1_s));

class SolarSystem: public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.description = "A demo of grand scales."
            " The Sun and planets radiuses, masses, orbital and rotational periods"
            " are all simulated to scale.";
        conf.worldDef = WorldDef{}.UseMaxVertexRadius(700000_km);
        conf.neededSettings = 0;
        conf.neededSettings |= (1u << NeedLinearSlopField);
        conf.neededSettings |= (1u << NeedCameraZoom);
        conf.neededSettings |= (1u << NeedDrawLabelsField);
        conf.neededSettings |= (1u << NeedMaxTranslation);
        conf.neededSettings |= (1u << NeedDeltaTime);
        conf.settings.linearSlop = 1000.0f;
        conf.settings.cameraZoom = 2.2e11f;
        conf.settings.drawLabels = true;
        conf.settings.maxTranslation = std::numeric_limits<float>::infinity();
        conf.settings.minDt =  1 * 3.6e3f; // 1 hour
        conf.settings.dt =    24 * 3.6e3f; // 1 day
        conf.settings.maxDt = 96 * 3.6e3f;
        return conf;
    }
    
    SolarSystem(): Test(GetTestConf())
    {
        m_world.SetGravity(LinearAcceleration2D{});
        const auto DynamicBD = BodyDef{}.UseType(BodyType::Dynamic);
        for (auto& sso: SolarSystemBodies)
        {
            const auto p = sso.orbitalPeriod;
            const auto c = sso.aveDist * Pi * 2;
            const auto n = &sso - SolarSystemBodies;
            const auto odds = n % 2;
            const auto l = Length2D{odds? -sso.aveDist: sso.aveDist, 0_m};
            const auto v = (p != 0_s)? (c / p) * (odds? -1: +1): 0_mps;
            const auto b = m_world.CreateBody(BodyDef{DynamicBD}.UseLocation(l));
            const auto a = 2 * Pi * 1_rad / sso.rotationalPeriod;
            b->SetVelocity(Velocity{LinearVelocity2D{0_mps, v}, a});
            const auto d = sso.mass / (Pi * Square(sso.radius));
            const auto sconf = DiskShape::Conf{}.UseVertexRadius(sso.radius).UseDensity(d);
            const auto shape = std::make_shared<DiskShape>(sconf);
            b->CreateFixture(shape);
        }
    }
    
    void PreStep(const Settings& settings, Drawer&) override
    {
        const auto bodies = m_world.GetBodies();
        for (auto it = std::begin(bodies); it != std::end(bodies); it = std::next(it))
        {
            const auto& b1 = *it;
            const auto m1 = GetMass(*b1);
            const auto loc1 = GetLocation(*b1);
            auto sumForce = Force2D{};
            for (auto jt = std::begin(bodies); jt != std::end(bodies); jt = std::next(jt))
            {
                const auto& b2 = *jt;
                if (b1 == b2)
                {
                    continue;
                }
                const auto m2 = GetMass(*b2);
                const auto delta = GetLocation(*b2) - loc1;
                const auto dir = GetUnitVector(delta);
                const auto rr = GetLengthSquared(delta);
                const auto orderedMass = std::minmax(m1, m2);
                const auto f = (G * orderedMass.first) * (orderedMass.second / rr);
                sumForce += f * dir;
            }
            // F = m a... i.e.  a = F / m.
            const auto linearAcceleration = sumForce / m1;
            SetLinearAcceleration(*b1, linearAcceleration);
            const auto v = GetVelocity(*b1, settings.dt * 1_s, MovementConf{
                std::numeric_limits<Length>::infinity(),
                std::numeric_limits<Angle>::infinity()
            });
            const auto stepDist = v.linear * settings.dt * 1_s;
            const auto location = GetLocation(*b1);
            const auto newLoc = location + stepDist;
            if (newLoc == location && stepDist != Length2D{})
            {
                //std::cout << "no move\n";
            }
        }
    }
};

} // namespace playrho

#endif /* SolarSystem_hpp */
