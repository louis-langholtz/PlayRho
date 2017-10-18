/*
 * Original work Copyright (c) 2008-2014 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_HEAVY_ON_LIGHT_HPP
#define  PLAYRHO_HEAVY_ON_LIGHT_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class HeavyOnLight : public Test
{
public:
    
    HeavyOnLight()
    {
        const auto bd = BodyDef{}.UseType(BodyType::Dynamic);
        const auto upperBodyDef = BodyDef(bd).UseLocation(Vec2(0.0f, 6.0f) * Meter);
        const auto lowerBodyDef = BodyDef(bd).UseLocation(Vec2(0.0f, 0.5f) * Meter);
        
        const auto groundConf = EdgeShape::Conf{}
            .UseVertex1(Vec2(-40.0f, 0.0f) * Meter)
            .UseVertex2(Vec2(40.0f, 0.0f) * Meter);
        
        const auto diskConf = DiskShape::Conf{}.UseDensity(Real(10) * KilogramPerSquareMeter);
        const auto smallerDiskConf = DiskShape::Conf(diskConf).UseVertexRadius(Real{0.5f} * Meter);
        const auto biggerDiskConf = DiskShape::Conf(diskConf).UseVertexRadius(Real{5.0f} * Meter);

        const auto ground = m_world.CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(groundConf));
        
        const auto lowerBody = m_world.CreateBody(lowerBodyDef);
        const auto upperBody = m_world.CreateBody(upperBodyDef);

        lowerBody->CreateFixture(std::make_shared<DiskShape>(smallerDiskConf));
        m_top = upperBody->CreateFixture(std::make_shared<DiskShape>(biggerDiskConf));
        
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "increase density of top shape", [&](KeyActionMods) {
            ChangeDensity(+KilogramPerSquareMeter);
        });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0, "decrease density of top shape", [&](KeyActionMods) {
            ChangeDensity(-KilogramPerSquareMeter);
        });
    }

    void ChangeDensity(Density change)
    {
        const auto oldDensity = m_top->GetShape()->GetDensity();
        const auto newDensity = std::max(oldDensity + change, KilogramPerSquareMeter);
        if (newDensity != oldDensity)
        {
            auto selectedFixtures = GetSelectedFixtures();
            const auto selectedFixture = selectedFixtures.size() == 1? selectedFixtures[0]: nullptr;
            const auto wasSelected = selectedFixture == m_top;
            const auto body = m_top->GetBody();
            body->DestroyFixture(m_top);
            auto conf = DiskShape::Conf{};
            conf.vertexRadius = Real{5.0f} * Meter;
            conf.density = newDensity;
            m_top = body->CreateFixture(std::make_shared<DiskShape>(conf));
            if (wasSelected)
            {
                selectedFixtures[0] = m_top;
                SetSelectedFixtures(selectedFixtures);
            }
        }
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        std::stringstream stream;
        stream << "Density of top shape: ";
        stream << double(Real{m_top->GetShape()->GetDensity() / KilogramPerSquareMeter});
        stream << " kg/m^2.";
        m_status = stream.str();
    }

    Fixture* m_top = nullptr;
};

} // namespace playrho

#endif
