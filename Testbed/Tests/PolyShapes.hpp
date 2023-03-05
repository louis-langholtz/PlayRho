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

#ifndef PLAYRHO_POLY_SHAPES_HPP
#define PLAYRHO_POLY_SHAPES_HPP

#include "../Framework/Test.hpp"

#include <vector>
#include <cstring>

namespace testbed {

/// This tests stacking. It also shows how to use World::Query and TestOverlap.
class PolyShapes : public Test
{
public:

    enum
    {
        e_maxBodies = 256
    };

    PolyShapes()
    {
        // Ground body
        CreateFixture(GetWorld(), CreateBody(GetWorld()), Shape{
            EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}
        });

        auto conf = PolygonShapeConf{};
        conf.UseDensity(1_kgpm2);
        conf.UseFriction(Real(0.3f));
        conf.Set({Vec2(-0.5f, 0.0f) * 1_m, Vec2(0.5f, 0.0f) * 1_m, Vec2(0.0f, 1.5f) * 1_m});
        m_polygons[0] = Shape(conf);
        conf.Set({Vec2(-0.1f, 0.0f) * 1_m, Vec2(0.1f, 0.0f) * 1_m, Vec2(0.0f, 1.5f) * 1_m});
        m_polygons[1] = Shape(conf);
        {
            const auto w = Real(1);
            const auto b = w / (2.0f + sqrt(2.0f));
            const auto s = sqrt(2.0f) * b;

            conf.Set({
                Vec2(0.5f * s, 0.0f) * 1_m,
                Vec2(0.5f * w, b) * 1_m,
                Vec2(0.5f * w, b + s) * 1_m,
                Vec2(0.5f * s, w) * 1_m,
                Vec2(-0.5f * s, w) * 1_m,
                Vec2(-0.5f * w, b + s) * 1_m,
                Vec2(-0.5f * w, b) * 1_m,
                Vec2(-0.5f * s, 0.0f) * 1_m
            });
            m_polygons[2] = Shape(conf);
        }
        conf.SetAsBox(0.5_m, 0.5_m);
        m_polygons[3] = Shape(conf);

        m_bodyIndex = 0;
        std::memset(m_bodies, 0, sizeof(m_bodies));
        
        RegisterForKey(GLFW_KEY_1, GLFW_PRESS, 0, "drop triangle", [&](KeyActionMods kam) {
            Create(kam.key - GLFW_KEY_1);
        });
        RegisterForKey(GLFW_KEY_2, GLFW_PRESS, 0, "drop thin triangle", [&](KeyActionMods kam) {
            Create(kam.key - GLFW_KEY_1);
        });
        RegisterForKey(GLFW_KEY_3, GLFW_PRESS, 0, "drop octagon", [&](KeyActionMods kam) {
            Create(kam.key - GLFW_KEY_1);
        });
        RegisterForKey(GLFW_KEY_4, GLFW_PRESS, 0, "drop square", [&](KeyActionMods kam) {
            Create(kam.key - GLFW_KEY_1);
        });
        RegisterForKey(GLFW_KEY_5, GLFW_PRESS, 0, "drop disk", [&](KeyActionMods kam) {
            Create(kam.key - GLFW_KEY_1);
        });
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "(de)activate some bodies", [&](KeyActionMods) {
            for (auto i = 0; i < e_maxBodies; i += 2)
            {
                if (IsValid(m_bodies[i]))
                {
                    const auto enabled = IsEnabled(GetWorld(), m_bodies[i]);
                    SetEnabled(GetWorld(), m_bodies[i], !enabled);
                }
            }
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "destroy a body", [&](KeyActionMods) {
            DestroyBodies();
        });
    }

    void Create(int index)
    {
        if (IsValid(m_bodies[m_bodyIndex]))
        {
            Destroy(GetWorld(), m_bodies[m_bodyIndex]);
            m_bodies[m_bodyIndex] = InvalidBodyID;
        }

        BodyConf bd;
        bd.type = BodyType::Dynamic;
        bd.linearAcceleration = GetGravity();

        const auto x = RandomFloat(-2.0f, 2.0f);
        bd.location = Vec2(x, 10.0f) * 1_m;
        bd.angle = 1_rad * RandomFloat(-Pi, Pi);

        if (index == 4)
        {
            bd.angularDamping = 0.02_Hz;
        }

        m_bodies[m_bodyIndex] = CreateBody(GetWorld(), bd);

        if (index < 4)
        {
            CreateFixture(GetWorld(), m_bodies[m_bodyIndex], m_polygons[index]);
        }
        else
        {
            CreateFixture(GetWorld(), m_bodies[m_bodyIndex], m_circle);
        }

        m_bodyIndex = GetModuloNext(m_bodyIndex, static_cast<decltype(m_bodyIndex)>(e_maxBodies));
    }

    void DestroyBodies()
    {
        for (auto i = 0; i < e_maxBodies; ++i)
        {
            if (IsValid(m_bodies[i]))
            {
                Destroy(GetWorld(), m_bodies[i]);
                m_bodies[i] = InvalidBodyID;
                return;
            }
        }
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        auto circleConf = DiskShapeConf{};
        circleConf.location = Vec2(0.0f, 1.1f) * 1_m;
        circleConf.vertexRadius = 2_m;

        const auto transform = Transform_identity;
        constexpr auto e_maxCount = 4;
        int count = 0;
        const auto circleChild = GetChild(circleConf, 0);
        const auto aabb = ComputeAABB(circleChild, transform);
        // Finds all the fixtures that overlap an AABB. Of those, we use TestOverlap to
        // determine which fixtures overlap a circle. Up to 4 overlapped fixtures will be
        // highlighted with a yellow border.
        Query(aabb, [&](FixtureID f, ChildCounter) {
            if (count < e_maxCount)
            {
                const auto xfm = GetTransformation(GetWorld(), f);
                const auto shape = GetShape(GetWorld(), f);
                const auto overlap = TestOverlap(GetChild(shape, 0), xfm, circleChild, transform);
                if (overlap >= 0_m2)
                {
                    ++count;
                    const auto overlapColor = Color(0.95f, 0.95f, 0.6f);
                    const auto type = GetType(shape);
                    const auto body = GetBody(GetWorld(), f);
                    if (type == GetTypeID<DiskShapeConf>()) {
                        const auto conf = TypeCast<DiskShapeConf>(shape);
                        const auto center = Transform(GetLocation(GetWorld(), body), xfm);
                        drawer.DrawCircle(center, conf.GetRadius(), overlapColor);
                    }
                    else if (type == GetTypeID<PolygonShapeConf>()) {
                        const auto conf = TypeCast<PolygonShapeConf>(shape);
                        const auto vertexCount = conf.GetVertexCount();
                        auto vertices = std::vector<Length2>(vertexCount);
                        for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
                        {
                            vertices[i] = Transform(conf.GetVertex(i), xfm);
                        }
                        drawer.DrawPolygon(&vertices[0], vertexCount, overlapColor);
                    }
                }
                return true;
            }
            return false;
        });

        const auto color = Color(0.4f, 0.7f, 0.8f);
        drawer.DrawCircle(circleConf.GetLocation(), circleConf.GetRadius(), color);
    }

    int m_bodyIndex;
    BodyID m_bodies[e_maxBodies];
    Shape m_polygons[4] = {
        Shape{PolygonShapeConf{}}, Shape{PolygonShapeConf{}},
        Shape{PolygonShapeConf{}}, Shape{PolygonShapeConf{}}
    };
    Shape m_circle = Shape{DiskShapeConf{}
        .UseRadius(0.5_m).UseDensity(1_kgpm2).UseFriction(Real(0.3))};
};

} // namespace testbed

#endif
