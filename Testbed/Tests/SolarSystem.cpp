/*
 * Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <sstream> // for std::ostringstream

namespace testbed {

/// @brief Solar system demo.
/// @see https://en.wikipedia.org/wiki/Solar_System

struct SolarSystemObject {
    const char* name;
    Length radius;
    Mass mass;
    Time orbitalPeriod; ///< Orbital period.
    Length aveDist; ///< Average distance.
    Time rotationalPeriod; ///< Rotational period.
};

// clang-format off
static const SolarSystemObject SolarSystemBodies[] = {
    { "The Sun", 696342_km, 1988550000.0_Yg,     0.000_d,    0_Gm,   25.050_d },
    { "Mercury",   2439_km,        330.2_Yg,    87.969_d,   57_Gm,   58.646_d },
    { "Venus",     6051_km,       4868.5_Yg,   224.701_d,  108_Gm, -243.025_d },
    { "Earth",     6371_km,       5973.6_Yg,   365.256_d,  150_Gm,    0.997_d },
    { "Mars",      3389_km,        641.8_Yg,   686.971_d,  230_Gm,    1.025_d },
    { "Jupiter",  69911_km,    1898600.0_Yg,  4332.590_d,  778_Gm,    0.413_d },
    { "Saturn",   58232_km,     568460.0_Yg, 10759.220_d, 1430_Gm,    0.439_d },
    { "Uranus",   25362_km,      86832.0_Yg, 30688.500_d, 2880_Gm,   -0.718_d },
    { "Neptune",  24622_km,     102430.0_Yg, 60182.000_d, 4500_Gm,    0.671_d },
};
// clang-format on

class SolarSystem : public Test
{
public:
    static inline const auto registered = RegisterTest("Solar System", MakeUniqueTest<SolarSystem>);

    static Test::Conf GetTestConf()
    {
        auto minRadius = std::numeric_limits<Length>::max();
        auto minName = "";
        auto maxRadius = std::numeric_limits<Length>::min();
        auto maxName = "";

        for (auto& sso : SolarSystemBodies) {
            if (minRadius > sso.radius) {
                minRadius = sso.radius;
                minName = sso.name;
            }
            if (maxRadius < sso.radius) {
                maxRadius = sso.radius;
                maxName = sso.name;
            }
        }

        std::ostringstream os;
        os << "A demo of grand scales!";
        os << " The Sun and planets radiuses, masses, orbital and rotational periods";
        os << " are all simulated to scale.";
        os << " Radiuses range from ";
        os << static_cast<float>(Real{minRadius / 1_km});
        os << " km (" << minName << "), to ";
        os << static_cast<float>(Real{maxRadius / 1_km});
        os << " km (" << maxName << ").";
        if (std::is_same_v<Real, float>) {
            os << "\n\n";
            os << "Note: recompile with playrho::Real set to use double (or bigger)";
            os << " for collisions to work better at these scales.";
        }

        auto conf = Test::Conf{};
        conf.description = os.str();
        conf.worldConf = WorldConf{}.UseVertexRadius({2_m, 1e7_km});
        conf.neededSettings = 0;
        conf.neededSettings |= (1u << NeedLinearSlopField);
        conf.neededSettings |= (1u << NeedCameraZoom);
        conf.neededSettings |= (1u << NeedDrawLabelsField);
        conf.neededSettings |= (1u << NeedMaxTranslation);
        conf.neededSettings |= (1u << NeedDeltaTime);
        conf.settings.linearSlop = 200 * 1000.0f; // 200_km;
        conf.settings.cameraZoom = 2.2e11f;
        conf.settings.drawLabels = true;
        conf.settings.maxTranslation = std::numeric_limits<float>::infinity();
        conf.settings.minDt = 1 * 3.6e3f; // 1 hour
        conf.settings.dt = 24 * 3.6e3f; // 1 day
        conf.settings.maxDt = 96 * 3.6e3f; // 4 days
        return conf;
    }

    SolarSystem() : Test(GetTestConf())
    {
        SetBombRadius(100_km);
        SetBombDensity(2e12_kgpm2);
        const auto DynamicBD = BodyConf{}.UseType(BodyType::Dynamic).UseBullet(true);
        for (auto& sso : SolarSystemBodies) {
            const auto p = sso.orbitalPeriod;
            const auto c = sso.aveDist * Pi * 2;
            const auto n = &sso - SolarSystemBodies;
            const auto odds = n % 2;
            const auto l = Length2{odds ? -sso.aveDist : sso.aveDist, 0_m};
            const auto v = (p != 0_s) ? (c / p) * (odds ? -1 : +1) : 0_mps;
            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            const auto b = CreateBody(GetWorld(), BodyConf(DynamicBD).UseLocation(l));
            const auto a = 2 * Pi * 1_rad / sso.rotationalPeriod;
            SetVelocity(GetWorld(), b, Velocity{LinearVelocity2{0_mps, v}, a});
            const auto d = sso.mass / (Pi * Square(sso.radius));
            const auto sconf = DiskShapeConf{}.UseRadius(sso.radius).UseDensity(d);
            Attach(GetWorld(), b, CreateShape(GetWorld(), sconf));
        }
        RegisterForKey(
            GLFW_KEY_EQUAL, GLFW_PRESS, 0, "Locks camera to following planet nearest mouse.",
            [&](KeyActionMods) { m_focalBody = FindClosestBody(GetWorld(), GetMouseWorld()); });
        RegisterForKey(GLFW_KEY_BACKSPACE, GLFW_PRESS, 0, "Unlock camera from following planet.",
                       [&](KeyActionMods) { m_focalBody = InvalidBodyID; });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, GLFW_MOD_SHIFT, "Increases bomb size.",
                       [&](KeyActionMods) { SetBombRadius(GetBombRadius() * 2); });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Decreases bomb size.",
                       [&](KeyActionMods) { SetBombRadius(GetBombRadius() / 2); });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, GLFW_MOD_SHIFT, "Increases bomb density.",
                       [&](KeyActionMods) { SetBombDensity(GetBombDensity() * 2); });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Decreases bomb density.",
                       [&](KeyActionMods) { SetBombDensity(GetBombDensity() / 2); });
    }

    void PreStep(const Settings&, Drawer&) override
    {
        SetAccelerations(GetWorld(), CalcGravitationalAcceleration);

        std::ostringstream os;
        if (IsValid(m_focalBody)) {
            const auto index = to_underlying(m_focalBody);
            os << "Camera locked on body " << index << ": ";
            os << SolarSystemBodies[index].name;
            os << ".";
        }
        else {
            os << "Camera unlocked from following any planet.";
        }
        os << " 'Bomb' size (radial) is now at " << Real{GetBombRadius() / 1_km} << "km.";
        os << " 'Bomb' density (areal) is now at " << Real{GetBombDensity() / 1_kgpm2} << "kg/m^2.";
        SetStatus(os.str());
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        if (IsValid(m_focalBody)) {
            drawer.SetTranslation(GetLocation(GetWorld(), m_focalBody));
        }
        auto& world = GetWorld();
        const auto bodies = GetBodies(world);
        std::for_each(begin(bodies), end(bodies), [&world](const auto& b) {
            SetAngle(world, b, GetNormalized(GetAngle(world, b)));
        });
    }

    BodyID m_focalBody = InvalidBodyID;
};

} // namespace testbed
