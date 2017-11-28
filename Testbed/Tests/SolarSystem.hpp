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

class SolarSystem: public Test
{
public:
    static Test::Conf GetTestConf()
    {
        auto minRadius = std::numeric_limits<Length>::max();
        auto minName = "";
        auto maxRadius = std::numeric_limits<Length>::min();
        auto maxName = "";

        for (auto& sso: SolarSystemBodies)
        {
            if (minRadius > sso.radius)
            {
                minRadius = sso.radius;
                minName = sso.name;
            }
            if (maxRadius < sso.radius)
            {
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
        if (std::is_same<Real, float>::value)
        {
            os << "\n\n";
            os << "Note: recompile with playrho::Real set to use double (or bigger)";
            os << " for collisions to work better at these scales.";
        }

        auto conf = Test::Conf{};
        conf.description = os.str();
        conf.worldDef = WorldDef{}.UseMaxVertexRadius(1e7_km);
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
        conf.settings.minDt =  1 * 3.6e3f; // 1 hour
        conf.settings.dt =    24 * 3.6e3f; // 1 day
        conf.settings.maxDt = 96 * 3.6e3f;
        return conf;
    }
    
    SolarSystem(): Test(GetTestConf())
    {
        m_bombRadius = 100_km;
        m_bombDensity = 2e12_kgpm2;
        m_world.SetGravity(LinearAcceleration2{});
        const auto DynamicBD = BodyDef{}.UseType(BodyType::Dynamic).UseBullet(true);
        for (auto& sso: SolarSystemBodies)
        {
            const auto p = sso.orbitalPeriod;
            const auto c = sso.aveDist * Pi * 2;
            const auto n = &sso - SolarSystemBodies;
            const auto odds = n % 2;
            const auto l = Length2{odds? -sso.aveDist: sso.aveDist, 0_m};
            const auto v = (p != 0_s)? (c / p) * (odds? -1: +1): 0_mps;
            // Use () instead of {} to avoid MSVC++ doing const preserving copy elision.
            const auto b = m_world.CreateBody(BodyDef(DynamicBD).UseLocation(l));
            const auto a = 2 * Pi * 1_rad / sso.rotationalPeriod;
            b->SetVelocity(Velocity{LinearVelocity2{0_mps, v}, a});
            const auto d = sso.mass / (Pi * Square(sso.radius));
            const auto sconf = DiskShape::Conf{}.UseVertexRadius(sso.radius).UseDensity(d);
            const auto shape = std::make_shared<DiskShape>(sconf);
            b->CreateFixture(shape);
        }
        RegisterForKey(GLFW_KEY_EQUAL, GLFW_PRESS, 0,
                       "Locks camera to following planet nearest mouse.",
                       [&](KeyActionMods) {
                           m_focalBody = FindClosestBody(m_world, GetMouseWorld());
                       });
        RegisterForKey(GLFW_KEY_BACKSPACE, GLFW_PRESS, 0,
                       "Unlock camera from following planet.",
                       [&](KeyActionMods) {
                           m_focalBody = nullptr;
                       });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, GLFW_MOD_SHIFT,
                       "Increases bomb size.",
                       [&](KeyActionMods) {
                           m_bombRadius *= 2;
                       });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0,
                       "Decreases bomb size.",
                       [&](KeyActionMods) {
                           m_bombRadius /= 2;;
                       });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, GLFW_MOD_SHIFT,
                       "Increases bomb density.",
                       [&](KeyActionMods) {
                           m_bombDensity *= 2;
                       });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0,
                       "Decreases bomb density.",
                       [&](KeyActionMods) {
                           m_bombDensity /= 2;;
                       });
    }
    
    void PreStep(const Settings&, Drawer& drawer) override
    {
        SetAccelerations(m_world, CalcGravitationalAcceleration);

        std::ostringstream os;
        if (m_focalBody)
        {
            const auto location = m_focalBody->GetLocation();
            drawer.SetTranslation(location);
            
            const auto index = GetWorldIndex(m_focalBody);
            os << "Camera locked on body " << index << ": ";
            os << SolarSystemBodies[index].name;
            os << ".";
        }
        else
        {
            os << "Camera unlocked from following any planet.";
        }
        os << " 'Bomb' size (radial) is now at " << Real{m_bombRadius/1_km} << "km.";
        os << " 'Bomb' density (areal) is now at " << Real{m_bombDensity/1_kgpm2} << "kg/m^2.";
        m_status = os.str();
    }
    
    const Body* m_focalBody = nullptr;
};

} // namespace playrho

#endif /* SolarSystem_hpp */
