/*
 * Original work Copyright (c) 2006-2014 Erin Catto http://www.box2d.org
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

#ifndef PLAYRHO_BASIC_SLIDER_CRANK_HPP
#define  PLAYRHO_BASIC_SLIDER_CRANK_HPP

#include "../Framework/Test.hpp"

namespace playrho {

// A basic slider crank created for GDC tutorial: Understanding Constraints
class BasicSliderCrank : public Test
{
public:
    BasicSliderCrank()
    {
        const auto ground = [&]()
        {
            BodyDef bd;
            bd.location = Vec2(0.0f, 17.0f) * Meter;
            return m_world.CreateBody(bd);
        }();
        
        {
            auto prevBody = ground;
            
            // Define crank.
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(-8.0f, 20.0f) * Meter;
                const auto body = m_world.CreateBody(bd);
                auto conf = PolygonShape::Conf{};
                conf.density = Real{2} * KilogramPerSquareMeter;
                body->CreateFixture(std::make_shared<PolygonShape>(Real{4} * Meter, Real{1} * Meter, conf));
                
                m_world.CreateJoint(RevoluteJointDef{prevBody, body, Vec2(-12.0f, 20.0f) * Meter});
                
                prevBody = body;
            }
            
            // Define connecting rod
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(4.0f, 20.0f) * Meter;
                const auto body = m_world.CreateBody(bd);
                auto conf = PolygonShape::Conf{};
                conf.density = Real{2} * KilogramPerSquareMeter;
                body->CreateFixture(std::make_shared<PolygonShape>(Real{8} * Meter, Real{1} * Meter, conf));
                
                m_world.CreateJoint(RevoluteJointDef{prevBody, body, Vec2(-4.0f, 20.0f) * Meter});
                
                prevBody = body;
            }
            
            // Define piston
            {
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.fixedRotation = true;
                bd.location = Vec2(12.0f, 20.0f) * Meter;
                const auto body = m_world.CreateBody(bd);
                auto conf = PolygonShape::Conf{};
                conf.density = Real{2} * KilogramPerSquareMeter;
                body->CreateFixture(std::make_shared<PolygonShape>(Real{3} * Meter, Real{3} * Meter, conf));
                
                m_world.CreateJoint(RevoluteJointDef{prevBody, body, Vec2(12.0f, 20.0f) * Meter});
                
                const PrismaticJointDef pjd{ground, body, Vec2(12.0f, 17.0f) * Meter, UnitVec2::GetRight()};
                m_world.CreateJoint(pjd);
            }
          }
    }
};

} // namespace playrho

#endif
