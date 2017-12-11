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

#ifndef PLAYRHO_COLLISION_PROCESSING_HPP
#define  PLAYRHO_COLLISION_PROCESSING_HPP

#include "../Framework/Test.hpp"
#include <algorithm>

namespace playrho {

// This test shows collision processing and tests
// deferred body destruction.
class CollisionProcessing : public Test
{
public:
    CollisionProcessing()
    {
        // Ground body
        {
            const auto ground = m_world.CreateBody();
            ground->CreateFixture(Shape{EdgeShape::Conf{Vec2(-50, 0) * 1_m, Vec2(50, 0) * 1_m}});
        }

        auto xLo = -5.0f, xHi = 5.0f;
        auto yLo = 2.0f, yHi = 35.0f;

        // Small triangle
        Length2 vertices[3];
        vertices[0] = Vec2(-1.0f, 0.0f) * 1_m;
        vertices[1] = Vec2(1.0f, 0.0f) * 1_m;
        vertices[2] = Vec2(0.0f, 2.0f) * 1_m;

        auto polygon = PolygonShape::Conf{};
        polygon.Set(Span<const Length2>{vertices, 3});
        polygon.SetDensity(1_kgpm2);

        BodyDef triangleBodyDef;
        triangleBodyDef.type = BodyType::Dynamic;
        triangleBodyDef.location = Vec2(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi)) * 1_m;

        const auto body1 = m_world.CreateBody(triangleBodyDef);
        body1->CreateFixture(Shape(polygon));

        // Large triangle (recycle definitions)
        vertices[0] *= 2.0f;
        vertices[1] *= 2.0f;
        vertices[2] *= 2.0f;
        polygon.Set(Span<const Length2>{vertices, 3});

        triangleBodyDef.location = Vec2(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi)) * 1_m;

        const auto body2 = m_world.CreateBody(triangleBodyDef);
        body2->CreateFixture(Shape(polygon));
        
        // Small box
        polygon.SetAsBox(1_m, 0.5_m);

        BodyDef boxBodyDef;
        boxBodyDef.type = BodyType::Dynamic;
        boxBodyDef.location = Vec2(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi)) * 1_m;

        const auto body3 = m_world.CreateBody(boxBodyDef);
        body3->CreateFixture(Shape(polygon));

        // Large box (recycle definitions)
        polygon.SetAsBox(2_m, 1_m);
        boxBodyDef.location = Vec2(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi)) * 1_m;
        
        const auto body4 = m_world.CreateBody(boxBodyDef);
        body4->CreateFixture(Shape(polygon));

        BodyDef circleBodyDef;
        circleBodyDef.type = BodyType::Dynamic;

        // Small circle
        circleBodyDef.location = Vec2(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi)) * 1_m;
        const auto body5 = m_world.CreateBody(circleBodyDef);
        body5->CreateFixture(Shape(DiskShapeConf{}.SetRadius(1_m).UseDensity(1_kgpm2)));

        // Large circle
        circleBodyDef.location = Vec2(RandomFloat(xLo, xHi), RandomFloat(yLo, yHi)) * 1_m;
        const auto body6 = m_world.CreateBody(circleBodyDef);
        body6->CreateFixture(Shape(DiskShapeConf{}.SetRadius(2_m).UseDensity(1_kgpm2)));
    }

    void PostStep(const Settings&, Drawer&) override
    {
        // We are going to destroy some bodies according to contact
        // points. We must buffer the bodies that should be destroyed
        // because they may belong to multiple contact points.
        const auto k_maxNuke = 6;
        Body* nuke[k_maxNuke];
        auto nukeCount = 0;

        // Traverse the contact results. Destroy bodies that
        // are touching heavier bodies.
        for (auto& point: GetPoints())
        {
            const auto body1 = point.fixtureA->GetBody();
            const auto body2 = point.fixtureB->GetBody();
            const auto mass1 = GetMass(*body1);
            const auto mass2 = GetMass(*body2);

            if (mass1 > 0_kg && mass2 > 0_kg)
            {
                if (mass2 > mass1)
                {
                    nuke[nukeCount++] = body1;
                }
                else
                {
                    nuke[nukeCount++] = body2;
                }

                if (nukeCount == k_maxNuke)
                {
                    break;
                }
            }
        }

        // Sort the nuke array to group duplicates.
        std::sort(nuke, nuke + nukeCount);

        // Destroy the bodies, skipping duplicates.
        auto i = 0;
        while (i < nukeCount)
        {
            const auto b = nuke[i++];
            while (i < nukeCount && nuke[i] == b)
            {
                ++i;
            }

            if (b != GetBomb())
            {
                m_world.Destroy(b);
            }
        }
    }
};

} // namespace playrho

#endif
