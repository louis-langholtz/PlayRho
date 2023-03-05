/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_COLLISION_FILTERING_HPP
#define  PLAYRHO_COLLISION_FILTERING_HPP

#include "../Framework/Test.hpp"

namespace testbed {

// This is a test of collision filtering.
// There is a triangle, a box, and a circle.
// There are 6 shapes. 3 large and 3 small.
// The 3 small ones always collide.
// The 3 large ones never collide.
// The boxes don't collide with triangles (except if both are small).
const Filter::index_type k_smallGroup = 1;
const Filter::index_type k_largeGroup = -1;

const Filter::bits_type k_defaultCategory = 0x0001;
const Filter::bits_type k_triangleCategory = 0x0002;
const Filter::bits_type k_boxCategory = 0x0004;
const Filter::bits_type k_circleCategory = 0x0008;

const Filter::bits_type k_triangleMask = 0xFFFF;
const Filter::bits_type k_boxMask = 0xFFFF ^ k_triangleCategory;
const Filter::bits_type k_circleMask = 0xFFFF;

class CollisionFiltering : public Test
{
public:
    CollisionFiltering()
    {
        // Ground body
        CreateFixture(GetWorld(), CreateBody(GetWorld()), Shape(EdgeShapeConf{}
            .UseFriction(0.3).Set(Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m)));
        
        // Small triangle
        Length2 vertices[3];
        vertices[0] = Vec2(-1.0f, 0.0f) * 1_m;
        vertices[1] = Vec2(1.0f, 0.0f) * 1_m;
        vertices[2] = Vec2(0.0f, 2.0f) * 1_m;
        auto polygon = PolygonShapeConf{};
        polygon.UseDensity(1_kgpm2);
        polygon.Set(vertices);

        auto triangleShapeConf = FixtureConf{};
        triangleShapeConf.filter.groupIndex = k_smallGroup;
        triangleShapeConf.filter.categoryBits = k_triangleCategory;
        triangleShapeConf.filter.maskBits = k_triangleMask;

        auto triangleBodyConf = BodyConf{};
        triangleBodyConf.type = BodyType::Dynamic;
        triangleBodyConf.location = Vec2(-5.0f, 2.0f) * 1_m;

        const auto body1 = CreateBody(GetWorld(), triangleBodyConf);
        CreateFixture(GetWorld(), body1, Shape(polygon), triangleShapeConf);

        // Large triangle (recycle definitions)
        vertices[0] *= 2.0f;
        vertices[1] *= 2.0f;
        vertices[2] *= 2.0f;
        polygon.Set(vertices);
        triangleShapeConf.filter.groupIndex = k_largeGroup;
        triangleBodyConf.location = Vec2(-5.0f, 6.0f) * 1_m;
        triangleBodyConf.fixedRotation = true; // look at me!

        const auto body2 = CreateBody(GetWorld(), triangleBodyConf);
        CreateFixture(GetWorld(), body2, Shape(polygon), triangleShapeConf);

        {
            auto bd = BodyConf{};
            bd.type = BodyType::Dynamic;
            bd.location = Vec2(-5.0f, 10.0f) * 1_m;
            const auto body = CreateBody(GetWorld(), bd);
            CreateFixture(GetWorld(), body, Shape{PolygonShapeConf{}.UseDensity(1_kgpm2).SetAsBox(0.5_m, 1_m)});

            auto jd = PrismaticJointConf{};
            jd.bodyA = body2;
            jd.bodyB = body;
            jd.enableLimit = true;
            jd.localAnchorA = Vec2(0.0f, 4.0f) * 1_m;
            jd.localAnchorB = Length2{};
            jd.localXAxisA = UnitVec::GetTop();
            jd.localYAxisA = GetRevPerpendicular(UnitVec::GetTop());
            jd.lowerTranslation = -1.0_m;
            jd.upperTranslation = +1.0_m;
            CreateJoint(GetWorld(), jd);
        }

        // Small box
        polygon.SetAsBox(1_m, 0.5_m);
        polygon.UseDensity(1_kgpm2);
        polygon.UseRestitution(Real(0.1));

        auto boxShapeConf = FixtureConf{};
        boxShapeConf.filter.groupIndex = k_smallGroup;
        boxShapeConf.filter.categoryBits = k_boxCategory;
        boxShapeConf.filter.maskBits = k_boxMask;

        auto boxBodyConf = BodyConf{};
        boxBodyConf.type = BodyType::Dynamic;
        boxBodyConf.location = Vec2(0.0f, 2.0f) * 1_m;

        const auto body3 = CreateBody(GetWorld(), boxBodyConf);
        CreateFixture(GetWorld(), body3, Shape(polygon), boxShapeConf);

        // Large box (recycle definitions)
        polygon.SetAsBox(2_m, 1_m);
        boxShapeConf.filter.groupIndex = k_largeGroup;
        boxBodyConf.location = Vec2(0.0f, 6.0f) * 1_m;

        const auto body4 = CreateBody(GetWorld(), boxBodyConf);
        CreateFixture(GetWorld(), body4, Shape(polygon), boxShapeConf);

        // Small circle
        auto circleConf = DiskShapeConf{};
        circleConf.density = 1_kgpm2;

        auto circleShapeConf = FixtureConf{};
        circleShapeConf.filter.groupIndex = k_smallGroup;
        circleShapeConf.filter.categoryBits = k_circleCategory;
        circleShapeConf.filter.maskBits = k_circleMask;

        auto circleBodyConf = BodyConf{};
        circleBodyConf.type = BodyType::Dynamic;
        circleBodyConf.location = Vec2(5.0f, 2.0f) * 1_m;
        
        const auto body5 = CreateBody(GetWorld(), circleBodyConf);
        circleConf.vertexRadius = 1_m;
        CreateFixture(GetWorld(), body5, Shape(circleConf), circleShapeConf);

        // Large circle
        circleShapeConf.filter.groupIndex = k_largeGroup;
        circleBodyConf.location = Vec2(5.0f, 6.0f) * 1_m;

        const auto body6 = CreateBody(GetWorld(), circleBodyConf);
        circleConf.vertexRadius = circleConf.vertexRadius * 2;
        CreateFixture(GetWorld(), body6, Shape(circleConf), circleShapeConf);
        
        SetAccelerations(GetWorld(), GetGravity());
    }
};
    
} // namespace testbed

#endif
