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

#ifndef BreakableTwo_hpp
#define BreakableTwo_hpp

#include "../Framework/Test.hpp"

namespace playrho {

class BreakableTwo: public Test
{
public:
    
    BreakableTwo()
    {
        const auto vr = 2 * DefaultLinearSlop;
        const auto polygonConf = PolygonShape::Conf{}.UseVertexRadius(vr);
        m_shape = std::make_shared<PolygonShape>(0.5f * Meter - vr, 0.5f * Meter - vr, polygonConf);
        m_shape->SetDensity(100 * KilogramPerSquareMeter);

        m_world->SetGravity(LinearAcceleration2D{});

        Body* bodies[20 * 20];
        const auto startLoc = Length2D{-10 * Meter, 10 * Meter};
        const auto bd = BodyDef{}.UseType(BodyType::Dynamic);
        for (auto y = 0; y < 20; ++y)
        {
            for (auto x = 0; x < 20; ++x)
            {
                const auto location = startLoc + Length2D{x * Meter, y * Meter};
                bodies[y * 20 + x] = m_world->CreateBody(BodyDef{bd}.UseLocation(location));
                bodies[y * 20 + x]->CreateFixture(m_shape);
                
                if (x > 0)
                {
                    const auto jd = WeldJointDef{
                        bodies[y * 20 + x - 1],
                        bodies[y * 20 + x],
                        location + Length2D{-0.5f * Meter, 0 * Meter}
                    };
                    m_world->CreateJoint(jd);
                }
                if (y > 0)
                {
                    const auto jd = WeldJointDef{
                        bodies[(y - 1) * 20 + x],
                        bodies[(y + 0) * 20 + x],
                        location + Length2D{0 * Meter, -0.5f * Meter}
                    };
                    m_world->CreateJoint(jd);
                }
            }
        }
    }

    void PostSolve(Contact& contact, const ContactImpulsesList& impulses,
                   ContactListener::iteration_type) override
    {
        if (!m_body)
        {
            // Should the body break?
            auto maxImpulse = GetMaxNormalImpulse(impulses);
            if (maxImpulse > Real{60} * Kilogram * MeterPerSecond)
            {
                const auto fA = contact.GetFixtureA();
                const auto fB = contact.GetFixtureB();
                if (fA->GetShape() == m_shape)
                {
                    m_body = fA->GetBody();
                }
                else if (fB->GetShape() == m_shape)
                {
                    m_body = fB->GetBody();
                }
            }
        }
    }
    
    void PreStep(const Settings&, Drawer&) override
    {
        if (m_body)
        {
            m_world->Destroy(m_body);
            m_body = nullptr;
        }
    }

private:
    std::shared_ptr<PolygonShape> m_shape;
    Body* m_body = nullptr;
};

} // namespace playrho

#endif /* BreakableTwo_hpp */
