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

#ifndef PLAYRHO_HEAVY_ON_LIGHT_TWO_HPP
#define  PLAYRHO_HEAVY_ON_LIGHT_TWO_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class HeavyOnLightTwo : public Test
{
public:
    
    HeavyOnLightTwo()
    {
        const auto ground = m_world.CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));
        
        const auto shape = std::make_shared<DiskShape>(Real{0.5f} * Meter);
        shape->SetDensity(Real{10} * KilogramPerSquareMeter);

        const auto body1 = m_world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(Vec2(0.0f, 2.5f) * Meter));
        body1->CreateFixture(shape);
        
        const auto body2 = m_world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(Vec2(0.0f, 3.5f) * Meter));
        body2->CreateFixture(shape);
        
        RegisterForKey(GLFW_KEY_H, GLFW_PRESS, 0, "Toggle Heavy", [&](KeyActionMods) {
            ToggleHeavy();
        });
    }
    
    void ToggleHeavy()
    {
        if (m_heavy)
        {
            m_world.Destroy(m_heavy);
            m_heavy = nullptr;
        }
        else
        {
            m_heavy = m_world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(Vec2(0.0f, 9.0f) * Meter));
            
            auto conf = DiskShape::Conf{};
            conf.density = Real{10} * KilogramPerSquareMeter;
            conf.vertexRadius = Real{5.0f} * Meter;
            m_heavy->CreateFixture(std::make_shared<DiskShape>(conf));
        }
    }
    
    Body* m_heavy = nullptr;
};

} // namespace playrho

#endif
