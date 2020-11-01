/*
 * Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_ORBITER_HPP
#define PLAYRHO_ORBITER_HPP

#include "../Framework/Test.hpp"

namespace testbed {
    
    class Orbiter: public Test
    {
    public:
        
        Orbiter()
        {
            SetGravity(LinearAcceleration2{});

            BodyConf bd;
            const auto radius = Real{12.0f};

            bd.type = BodyType::Static;
            bd.location = m_center;
            const auto ctrBody = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), ctrBody, Shape{DiskShapeConf{}.UseRadius(3_m)});

            bd.type = BodyType::Dynamic;
            bd.location = Length2{GetX(m_center), GetY(m_center) + radius * 1_m};
            m_orbiter = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), m_orbiter,
                          Shape{DiskShapeConf{}.UseRadius(0.5_m).UseDensity(1_kgpm2)});
            
            const auto velocity = Velocity{
                Vec2{Pi * radius / 2, 0} * 1_mps,
                360_deg / 1_s
            };
            SetVelocity(GetWorld(), m_orbiter, velocity);
            
            auto conf = ChainShapeConf{};
            conf.Set(GetCircleVertices(20_m, 180));
            conf.UseVertexRadius(0.1_m);
            conf.UseDensity(1_kgpm2);
            const auto outerCicle = Shape(conf);

            bd.type = BodyType::Dynamic;
            bd.location = m_center;
            bd.bullet = true;
            const auto dysonSphere = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), dysonSphere, outerCicle);
        }
        
        void PreStep(const Settings&, Drawer&) override
        {
            const auto force = GetCentripetalForce(GetWorld(), m_orbiter, m_center);
            const auto linAccel = force * GetInvMass(GetWorld(), m_orbiter);
            const auto angAccel = 0 * RadianPerSquareSecond;
            SetAcceleration(GetWorld(), m_orbiter, linAccel, angAccel);
        }
        
    private:
        BodyID m_orbiter = InvalidBodyID;
        Length2 const m_center = Vec2{0, 20} * 1_m;

    };
    
}

#endif /* PLAYRHO_ORBITER_HPP */
