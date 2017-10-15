/*
* Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_VARYING_FRICTION_HPP
#define PLAYRHO_VARYING_FRICTION_HPP

#include "../Framework/Test.hpp"

namespace playrho {

class VaryingFriction : public Test
{
public:

    VaryingFriction()
    {
        {
            const auto ground = m_world.CreateBody();
            ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter, Vec2(40.0f, 0.0f) * Meter));
        }

        const auto sliderPlank = std::make_shared<PolygonShape>(Real{13.0f} * Meter, Real{0.25f} * Meter);
        const auto sliderWall = std::make_shared<PolygonShape>(Real{0.25f} * Meter, Real{1.0f} * Meter);
        
        {
            BodyDef bd;
            bd.location = Vec2(-4.0f, 22.0f) * Meter;
            bd.angle = Real{-0.25f} * Radian;

            const auto ground = m_world.CreateBody(bd);
            ground->CreateFixture(sliderPlank);
        }

        {
            BodyDef bd;
            bd.location = Vec2(10.5f, 19.0f) * Meter;

            const auto ground = m_world.CreateBody(bd);
            ground->CreateFixture(sliderWall);
        }

        {
            BodyDef bd;
            bd.location = Vec2(4.0f, 14.0f) * Meter;
            bd.angle = Real{0.25f} * Radian;

            const auto ground = m_world.CreateBody(bd);
            ground->CreateFixture(sliderPlank);
        }

        {
            BodyDef bd;
            bd.location = Vec2(-10.5f, 11.0f) * Meter;

            const auto ground = m_world.CreateBody(bd);
            ground->CreateFixture(sliderWall);
        }

        {
            BodyDef bd;
            bd.location = Vec2(-4.0f, 6.0f) * Meter;
            bd.angle = Real{-0.25f} * Radian;

            const auto ground = m_world.CreateBody(bd);
            ground->CreateFixture(sliderPlank);
        }

        {
            auto shape = PolygonShape(Real{0.5f} * Meter, Real{0.5f} * Meter);
            shape.SetDensity(Real{25} * KilogramPerSquareMeter);

            float friction[5] = {std::sqrt(std::numeric_limits<float>::max()), 0.5f, 0.35f, 0.1f, 0.0f};
            for (auto i = 0; i < 5; ++i)
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(-15.0f + 4.0f * i, 28.0f) * Meter;
                const auto body = m_world.CreateBody(bd);

                shape.SetFriction(Real(friction[i]));
                body->CreateFixture(std::make_shared<PolygonShape>(shape));
            }
        }
    }
};

} // namespace playrho

#endif
