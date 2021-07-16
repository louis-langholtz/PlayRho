/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include "../Framework/Test.hpp"

#include <cstring>
#include <vector>

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

namespace testbed {

class RayCast : public Test
{
public:
    static inline const auto registered = RegisterTest("Ray-Cast", MakeUniqueTest<RayCast>);
    enum { e_maxBodies = 256 };
    enum class Mode { e_closest, e_any, e_multiple };

    RayCast()
    {
        m_edge = CreateShape(
            GetWorld(),
            EdgeShapeConf{Vec2(-1.0f, 0.0f) * 1_m, Vec2(1.0f, 0.0f) * 1_m}.UseFriction(Real(0.3)));
        m_circle = CreateShape(GetWorld(), DiskShapeConf{}.UseRadius(0.5_m).UseFriction(Real(0.3)));

        // Ground body
        Attach(GetWorld(), CreateBody(GetWorld()),
               Shape{EdgeShapeConf{Vec2(-40.0f, 0.0f) * 1_m, Vec2(40.0f, 0.0f) * 1_m}});

        auto conf = PolygonShapeConf{};
        conf.UseFriction(Real(0.3));
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
        }
        m_polygons[2] = CreateShape(GetWorld(), conf);
        conf.SetAsBox(0.5_m, 0.5_m);
        m_polygons[3] = CreateShape(GetWorld(), conf);
        std::fill(std::begin(m_bodies), std::end(m_bodies), InvalidBodyID);

        RegisterForKey(GLFW_KEY_1, GLFW_PRESS, 0,
                       "drop triangles that should be ignored by the ray.",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_2, GLFW_PRESS, 0,
                       "drop shape that should not be ignored by the ray.",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_3, GLFW_PRESS, 0,
                       "drop shape that should not be ignored by the ray.",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_4, GLFW_PRESS, 0,
                       "drop shape that should not be ignored by the ray.",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_5, GLFW_PRESS, 0,
                       "drop shape that should not be ignored by the ray.",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_6, GLFW_PRESS, 0,
                       "drop shape that should not be ignored by the ray.",
                       [&](KeyActionMods kam) { Create(kam.key - GLFW_KEY_1); });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Destroy Bodies",
                       [&](KeyActionMods) { DestroyBodies(); });
        RegisterForKey(GLFW_KEY_M, GLFW_PRESS, 0, "Change mode of the raycast test",
                       [&](KeyActionMods) {
                           if (m_mode == Mode::e_closest) {
                               m_mode = Mode::e_any;
                           }
                           else if (m_mode == Mode::e_any) {
                               m_mode = Mode::e_multiple;
                           }
                           else if (m_mode == Mode::e_multiple) {
                               m_mode = Mode::e_closest;
                           }
                       });
    }

    void Create(int type)
    {
        if (IsValid(m_bodies[m_bodyIndex])) {
            Destroy(GetWorld(), m_bodies[m_bodyIndex]);
            m_bodies[m_bodyIndex] = InvalidBodyID;
        }
        auto bd = BodyConf{};
        const auto x = RandomFloat(-10.0f, 10.0f);
        const auto y = RandomFloat(0.0f, 20.0f);
        bd.location = Vec2(x, y) * 1_m;
        bd.angle = 1_rad * RandomFloat(-Pi, Pi);
        if (type == 4) {
            bd.angularDamping = 0.02_Hz;
        }
        m_bodies[m_bodyIndex] = CreateBody(GetWorld(), bd);
        m_userData.resize(m_bodies[m_bodyIndex].get() + 1u, -1);
        m_userData[m_bodies[m_bodyIndex].get()] = type;
        if (type < 4) {
            Attach(GetWorld(), m_bodies[m_bodyIndex], m_polygons[type]);
        }
        else if (type < 5) {
            Attach(GetWorld(), m_bodies[m_bodyIndex], m_circle);
        }
        else {
            Attach(GetWorld(), m_bodies[m_bodyIndex], m_edge);
        }
        m_bodyIndex = GetModuloNext(m_bodyIndex, static_cast<decltype(m_bodyIndex)>(e_maxBodies));
    }

    void DestroyBodies()
    {
        for (auto i = 0; i < e_maxBodies; ++i) {
            if (IsValid(m_bodies[i])) {
                Destroy(GetWorld(), m_bodies[i]);
                m_bodies[i] = InvalidBodyID;
                return;
            }
        }
    }

    static const char* GetModeName(Mode mode) noexcept
    {
        switch (mode) {
        case Mode::e_closest:
            return "closest - find closest fixture along the ray";
        case Mode::e_any:
            return "any - check for obstruction";
        case Mode::e_multiple:
            return "multiple - gather multiple fixtures";
        }
        return "unknown";
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        std::stringstream stream;
        stream << "Mode of the raycast test currently: ";
        stream << GetModeName(m_mode);
        stream << ".";
        SetStatus(stream.str());

        const auto L = 11.0f;
        const auto point1 = Vec2(0.0f, 10.0f) * 1_m;
        const auto d = Vec2(L * cos(m_angle), L * sin(m_angle)) * 1_m;
        const auto point2 = point1 + d;

        if (m_mode == Mode::e_closest) {
            auto hit = false;
            Length2 point;
            UnitVec normal;

            d2::RayCast(GetWorld(), RayCastInput{point1, point2, Real{1}},
                        [&](BodyID b, ShapeID, ChildCounter, const Length2& p, const UnitVec& n) {
                            const auto type =
                                (b.get() < m_userData.size()) ? m_userData[b.get()] : 0;
                            if (type == 0) {
                                // Instruct the calling code to ignore this fixture and
                                // continue the ray-cast to the next fixture.
                                return RayCastOpcode::IgnoreFixture;
                            }
                            hit = true;
                            point = p;
                            normal = n;
                            // Instruct the calling code to clip the ray and
                            // continue the ray-cast to the next fixture. WARNING: do not assume
                            // that fixtures are reported in order. However, by clipping, we can
                            // always get the closest fixture.
                            return RayCastOpcode::ClipRay;
                        });

            if (hit) {
                drawer.DrawPoint(point, 5.0f, Color(0.4f, 0.9f, 0.4f));
                drawer.DrawSegment(point1, point, Color(0.8f, 0.8f, 0.8f));
                const auto head = point + Real{0.5f} * normal * 1_m;
                drawer.DrawSegment(point, head, Color(0.9f, 0.9f, 0.4f));
            }
            else {
                drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
            }
        }
        else if (m_mode == Mode::e_any) {
            auto hit = false;
            Length2 point;
            UnitVec normal;

            // This callback finds any hit. Polygon 0 is filtered. For this type of query we are
            // just checking for obstruction, so the actual fixture and hit point are irrelevant.
            d2::RayCast(GetWorld(), RayCastInput{point1, point2, Real{1}},
                        [&](BodyID b, ShapeID, ChildCounter, const Length2& p, const UnitVec& n) {
                            const auto type =
                                (b.get() < m_userData.size()) ? m_userData[b.get()] : 0;
                            if (type == 0) {
                                // Instruct the calling code to ignore this fixture and
                                // continue the ray-cast to the next fixture.
                                return RayCastOpcode::IgnoreFixture;
                            }
                            // At this point we have a hit, so we know the ray is obstructed.
                            // Instruct the calling code to terminate the ray-cast.
                            hit = true;
                            point = p;
                            normal = n;
                            return RayCastOpcode::Terminate;
                        });

            if (hit) {
                drawer.DrawPoint(point, 5.0f, Color(0.4f, 0.9f, 0.4f));
                drawer.DrawSegment(point1, point, Color(0.8f, 0.8f, 0.8f));
                const auto head = point + Real{0.5f} * normal * 1_m;
                drawer.DrawSegment(point, head, Color(0.9f, 0.9f, 0.4f));
            }
            else {
                drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
            }
        }
        else if (m_mode == Mode::e_multiple) {
            drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));

            // This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
            // The fixtures are not necessary reported in order, so we might not capture
            // the closest fixture.
            d2::RayCast(GetWorld(), RayCastInput{point1, point2, Real{1}},
                        [&](BodyID b, ShapeID, ChildCounter, const Length2& p, const UnitVec& n) {
                            const auto type =
                                (b.get() < m_userData.size()) ? m_userData[b.get()] : 0;
                            if (type == 0) {
                                // Instruct the calling code to ignore this fixture
                                // and continue the ray-cast to the next fixture.
                                return RayCastOpcode::IgnoreFixture;
                            }
                            drawer.DrawPoint(p, 5.0f, Color(0.4f, 0.9f, 0.4f));
                            drawer.DrawSegment(point1, p, Color(0.8f, 0.8f, 0.8f));
                            const auto head = p + Real{0.5f} * n * 1_m;
                            drawer.DrawSegment(p, head, Color(0.9f, 0.9f, 0.4f));
                            // Instruct the caller to continue without clipping the ray.
                            return RayCastOpcode::ResetRay;
                        });
        }

        const auto advanceRay = !settings.pause || settings.singleStep;
        if (advanceRay) {
            m_angle += 0.25f * Pi / 180.0f;
        }

#if 0
        // This case was failing.
        {
            Vec2 vertices[4];
            //vertices[0] = Vec2(-22.875f, -3.0f);
            //vertices[1] = Vec2(22.875f, -3.0f);
            //vertices[2] = Vec2(22.875f, 3.0f);
            //vertices[3] = Vec2(-22.875f, 3.0f);

            PolygonShapeConf shape;
            //shape.Set(vertices, 4);
            shape.SetAsBox(22.875f, 3.0f);

            RayCastInput input;
            input.p1 = Vec2(10.2725f,1.71372f);
            input.p2 = Vec2(10.2353f,2.21807f);
            //input.maxFraction = 0.567623f;
            input.maxFraction = 0.56762173f;

            Transformation xf;
            xf.SetIdentity();
            xf.position = Vec2(23.0f, 5.0f);

            RayCastHit output;
            bool hit;
            hit = shape.RayCast(&output, input, xf);
            hit = false;

            Color color(1.0f, 1.0f, 1.0f);
            Vec2 vs[4];
            for (auto i = 0; i < 4; ++i)
            {
                vs[i] = Transform(shape.m_vertices[i], xf);
            }

            drawer.DrawPolygon(vs, 4, color);
            drawer.DrawSegment(input.p1, input.p2, color);
        }
#endif
    }

    int m_bodyIndex = 0;
    BodyID m_bodies[e_maxBodies];
    std::vector<int> m_userData;
    ShapeID m_polygons[4] = {InvalidShapeID, InvalidShapeID, InvalidShapeID, InvalidShapeID};
    ShapeID m_circle = InvalidShapeID;
    ShapeID m_edge = InvalidShapeID;
    Real m_angle = 0.0f;
    Mode m_mode = Mode::e_closest;
};

} // namespace testbed
