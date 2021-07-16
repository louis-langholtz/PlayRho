/*
 * Original work Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#ifndef PLAYRHO_EDGE_SHAPES_HPP
#define PLAYRHO_EDGE_SHAPES_HPP

#include "../Framework/Test.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace testbed {

class EdgeShapes : public Test
{
public:
    static inline const auto registered = RegisterTest("Edge Shapes", MakeUniqueTest<EdgeShapes>);
    enum { e_maxBodies = 256 };

    EdgeShapes()
    {
        m_circle = CreateShape(
            GetWorld(),
            DiskShapeConf{}.UseRadius(0.5_m).UseFriction(Real(0.3)).UseDensity(20_kgpm2));
        // Ground body
        {
            const auto ground = CreateBody(GetWorld());
            auto x1 = -20.0f;
            auto y1 = 2.0f * std::cos(x1 / 10.0f * static_cast<float>(Pi));
            for (auto i = 0; i < 80; ++i) {
                const auto x2 = x1 + 0.5f;
                const auto y2 = 2.0f * std::cos(x2 / 10.0f * static_cast<float>(Pi));
                Attach(
                    GetWorld(), ground,
                    CreateShape(GetWorld(), EdgeShapeConf{Vec2(x1, y1) * 1_m, Vec2(x2, y2) * 1_m}));
                x1 = x2;
                y1 = y2;
            }
        }

        auto conf = PolygonShapeConf{};
        conf.UseFriction(Real(0.3));
        conf.UseDensity(20_kgpm2);
        conf.Set({Vec2(-0.5f, 0.0f) * 1_m, Vec2(0.5f, 0.0f) * 1_m, Vec2(0.0f, 1.5f) * 1_m});
        m_polygons[0] = CreateShape(GetWorld(), conf);

        conf.Set({Vec2(-0.1f, 0.0f) * 1_m, Vec2(0.1f, 0.0f) * 1_m, Vec2(0.0f, 1.5f) * 1_m});
        m_polygons[1] = CreateShape(GetWorld(), conf);

        {
            const auto w = 1.0f;
            const auto b = w / (2.0f + sqrt(2.0f));
            const auto s = sqrt(2.0f) * b;

            conf.Set({Vec2(0.5f * s, 0.0f) * 1_m, Vec2(0.5f * w, b) * 1_m,
                      Vec2(0.5f * w, b + s) * 1_m, Vec2(0.5f * s, w) * 1_m,
                      Vec2(-0.5f * s, w) * 1_m, Vec2(-0.5f * w, b + s) * 1_m,
                      Vec2(-0.5f * w, b) * 1_m, Vec2(-0.5f * s, 0.0f) * 1_m});
            m_polygons[2] = CreateShape(GetWorld(), conf);
        }

        conf.SetAsBox(0.5_m, 0.5_m);
        m_polygons[3] = CreateShape(GetWorld(), conf);

        std::fill(begin(m_bodies), end(m_bodies), InvalidBodyID);

        RegisterForKey(GLFW_KEY_1, GLFW_PRESS, 0, "to drop stuff",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_2, GLFW_PRESS, 0, "to drop stuff",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_3, GLFW_PRESS, 0, "to drop stuff",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_4, GLFW_PRESS, 0, "to drop stuff",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_5, GLFW_PRESS, 0, "to drop stuff",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "To Destroy Bodies",
                       [&](KeyActionMods) { DestroyBodies(); });
    }

    void Create(int index)
    {
        if (m_bodies[m_bodyIndex] != InvalidBodyID) {
            Destroy(GetWorld(), m_bodies[m_bodyIndex]);
            m_bodies[m_bodyIndex] = InvalidBodyID;
        }

        BodyConf bd;

        const auto x = RandomFloat(-10.0f, 10.0f);
        const auto y = RandomFloat(10.0f, 20.0f);
        bd.location = Vec2(x, y) * 1_m;
        bd.angle = 1_rad * RandomFloat(-Pi, Pi);
        bd.type = BodyType::Dynamic;
        bd.linearAcceleration = GetGravity();

        if (index == 4) {
            bd.angularDamping = 0.02_Hz;
        }

        m_bodies[m_bodyIndex] = CreateBody(GetWorld(), bd);

        if (index < 4) {
            Attach(GetWorld(), m_bodies[m_bodyIndex], m_polygons[index]);
        }
        else {
            Attach(GetWorld(), m_bodies[m_bodyIndex], m_circle);
        }

        m_bodyIndex = GetModuloNext(m_bodyIndex, static_cast<decltype(m_bodyIndex)>(e_maxBodies));
    }

    void DestroyBodies()
    {
        for (auto i = 0; i < e_maxBodies; ++i) {
            if (m_bodies[i] != InvalidBodyID) {
                Destroy(GetWorld(), m_bodies[i]);
                m_bodies[i] = InvalidBodyID;
                return;
            }
        }
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        const auto L = Real(25);
        const auto point1 = Vec2(0.0f, 10.0f) * 1_m;
        const auto d = Vec2(L * cos(m_angle), -L * abs(sin(m_angle))) * 1_m;
        const auto point2 = point1 + d;

        auto shapeId = InvalidShapeID;
        Length2 point;
        UnitVec normal;

        RayCast(GetWorld(), RayCastInput{point1, point2, Real{1}},
                [&](BodyID, ShapeID f, ChildCounter, Length2 p, UnitVec n) {
                    shapeId = f;
                    point = p;
                    normal = n;
                    return RayCastOpcode::ClipRay;
                });

        if (IsValid(shapeId)) {
            drawer.DrawPoint(point, 5.0f, Color(0.4f, 0.9f, 0.4f));
            drawer.DrawSegment(point1, point, Color(0.8f, 0.8f, 0.8f));
            const auto head = point + Real{0.5f} * normal * 1_m;
            drawer.DrawSegment(point, head, Color(0.9f, 0.9f, 0.4f));
        }
        else {
            drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
        }

        const auto advanceRay = !settings.pause || settings.singleStep;
        if (advanceRay) {
            m_angle += 0.25f * Pi / 180.0f;
        }
    }

    int m_bodyIndex = 0;
    BodyID m_bodies[e_maxBodies];
    ShapeID m_polygons[4] = {InvalidShapeID, InvalidShapeID, InvalidShapeID, InvalidShapeID};
    ShapeID m_circle = InvalidShapeID;
    Real m_angle = 0;
};

} // namespace testbed

#endif
