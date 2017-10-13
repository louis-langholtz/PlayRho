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

#ifndef PLAYRHO_COMPOUND_SHAPES_HPP
#define  PLAYRHO_COMPOUND_SHAPES_HPP

#include "../Framework/Test.hpp"

namespace playrho {

// TODO_ERIN test joints on compounds.
class CompoundShapes : public Test
{
public:
    CompoundShapes()
    {
        {
            BodyDef bd;
            bd.location = Vec2(0.0f, 0.0f) * Meter;
            const auto body = m_world->CreateBody(bd);
            body->CreateFixture(std::make_shared<EdgeShape>(Vec2(50.0f, 0.0f) * Meter, Vec2(-50.0f, 0.0f) * Meter));
        }

        {
            auto conf = DiskShape::Conf{};
            conf.vertexRadius = Real{0.5f} * Meter;
            
            conf.location = Vec2{-0.5f, 0.5f} * Meter;
            const auto circle1 = std::make_shared<DiskShape>(conf);
            circle1->SetDensity(Real{2} * KilogramPerSquareMeter);
            conf.location = Vec2{0.5f, 0.5f} * Meter;
            const auto circle2 = std::make_shared<DiskShape>(conf);
            for (auto i = 0; i < 10; ++i)
            {
                const auto x = RandomFloat(-0.1f, 0.1f);
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(x + 5.0f, 1.05f + 2.5f * i) * Meter;
                bd.angle = Radian * RandomFloat(-Pi, Pi);
                const auto body = m_world->CreateBody(bd);
                body->CreateFixture(circle1);
                body->CreateFixture(circle2);
            }
        }

        {
            const auto polygon1 = std::make_shared<PolygonShape>(Real{0.25f} * Meter, Real{0.5f} * Meter);
            polygon1->SetDensity(Real{2} * KilogramPerSquareMeter);
            auto polygon2 = std::make_shared<PolygonShape>();
            polygon2->SetDensity(Real{2} * KilogramPerSquareMeter);
            SetAsBox(*polygon2, Real{0.25f} * Meter, Real{0.5f} * Meter, Vec2(0.0f, -0.5f) * Meter, Real{0.5f} * Radian * Pi);
            for (int i = 0; i < 10; ++i)
            {
                const auto x = RandomFloat(-0.1f, 0.1f);
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(x - 5.0f, 1.05f + 2.5f * i) * Meter;
                bd.angle = Radian * RandomFloat(-Pi, Pi);
                const auto body = m_world->CreateBody(bd);
                body->CreateFixture(polygon1);
                body->CreateFixture(polygon2);
            }
        }

        {
            Transformation xf1;
            xf1.q = UnitVec2::Get(Real{0.3524f} * Radian * Pi);
            xf1.p = GetVec2(GetXAxis(xf1.q)) * Meter;

            auto triangle1 = std::make_shared<PolygonShape>();
            triangle1->Set(Span<const Length2D>{
                Transform(Vec2(-1.0f, 0.0f) * Meter, xf1),
                Transform(Vec2(1.0f, 0.0f) * Meter, xf1),
                Transform(Vec2(0.0f, 0.5f) * Meter, xf1)
            });
            triangle1->SetDensity(Real{2} * KilogramPerSquareMeter);

            Transformation xf2;
            xf2.q = UnitVec2::Get(Real{-0.3524f} * Radian * Pi);
            xf2.p = -GetVec2(GetXAxis(xf2.q)) * Meter;

            auto triangle2 = std::make_shared<PolygonShape>();
            triangle2->Set(Span<const Length2D>{
                Transform(Vec2(-1.0f, 0.0f) * Meter, xf2),
                Transform(Vec2(1.0f, 0.0f) * Meter, xf2),
                Transform(Vec2(0.0f, 0.5f) * Meter, xf2)
            });
            triangle2->SetDensity(Real{2} * KilogramPerSquareMeter);
            
            for (auto i = 0; i < 10; ++i)
            {
                const auto x = RandomFloat(-0.1f, 0.1f);
                BodyDef bd;
                bd.type = BodyType::Dynamic;
                bd.location = Vec2(x, 2.05f + 2.5f * i) * Meter;
                bd.angle = Real{0.0f} * Radian;
                const auto body = m_world->CreateBody(bd);
                body->CreateFixture(triangle1);
                body->CreateFixture(triangle2);
            }
        }

        {
            const auto bottom = std::make_shared<PolygonShape>(Real{1.5f} * Meter, Real{0.15f} * Meter);
            bottom->SetDensity(Real{4} * KilogramPerSquareMeter);

            auto left = std::make_shared<PolygonShape>();
            left->SetDensity(Real{4} * KilogramPerSquareMeter);
            SetAsBox(*left, Real{0.15f} * Meter, Real{2.7f} * Meter, Vec2(-1.45f, 2.35f) * Meter, Real{+0.2f} * Radian);

            auto right = std::make_shared<PolygonShape>();
            right->SetDensity(Real{4} * KilogramPerSquareMeter);
            SetAsBox(*right, Real{0.15f} * Meter, Real{2.7f} * Meter, Vec2(1.45f, 2.35f) * Meter, Real{-0.2f} * Radian);

            BodyDef bd;
            bd.type = BodyType::Dynamic;
            bd.location = Vec2( 0.0f, 2.0f ) * Meter;
            const auto body = m_world->CreateBody(bd);
            body->CreateFixture(bottom);
            body->CreateFixture(left);
            body->CreateFixture(right);
        }
    }
};

} // namespace playrho

#endif
