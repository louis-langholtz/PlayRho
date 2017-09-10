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

#ifndef PLAYRHO_RAY_CAST_HPP
#define PLAYRHO_RAY_CAST_HPP

#include "../Framework/Test.hpp"
#include <cstring>

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

namespace playrho {

class RayCast : public Test
{
public:

    enum
    {
        e_maxBodies = 256
    };

    enum class Mode
    {
        e_closest,
        e_any,
        e_multiple
    };

    RayCast()
    {
        m_circle->SetVertexRadius(Real{0.5f} * Meter);
        m_circle->SetFriction(Real(0.3f));
        m_edge->SetFriction(Real(0.3f));
        
        // Ground body
        const auto ground = m_world->CreateBody();
        ground->CreateFixture(std::make_shared<EdgeShape>(Vec2(-40.0f, 0.0f) * Meter,
                                                          Vec2(40.0f, 0.0f) * Meter));
        
        for (auto&& p: m_polygons)
        {
            p = std::make_shared<PolygonShape>();
            p->SetFriction(Real(0.3f));
        }

        m_polygons[0]->Set({
            Vec2(-0.5f, 0.0f) * Meter,
            Vec2(0.5f, 0.0f) * Meter,
            Vec2(0.0f, 1.5f) * Meter
        });
        m_polygons[1]->Set({
            Vec2(-0.1f, 0.0f) * Meter,
            Vec2(0.1f, 0.0f) * Meter,
            Vec2(0.0f, 1.5f) * Meter
        });

        {
            const auto w = 1.0f;
            const auto b = w / (2.0f + Sqrt(2.0f));
            const auto s = Sqrt(2.0f) * b;

            m_polygons[2]->Set({
                Vec2(0.5f * s, 0.0f) * Meter,
                Vec2(0.5f * w, b) * Meter,
                Vec2(0.5f * w, b + s) * Meter,
                Vec2(0.5f * s, w) * Meter,
                Vec2(-0.5f * s, w) * Meter,
                Vec2(-0.5f * w, b + s) * Meter,
                Vec2(-0.5f * w, b) * Meter,
                Vec2(-0.5f * s, 0.0f) * Meter
            });
        }
        m_polygons[3]->SetAsBox(Real{0.5f} * Meter, Real{0.5f} * Meter);
        std::memset(m_bodies, 0, sizeof(m_bodies));
    }

    void Create(int index)
    {
        if (m_bodies[m_bodyIndex])
        {
            m_world->Destroy(m_bodies[m_bodyIndex]);
            m_bodies[m_bodyIndex] = nullptr;
        }

        BodyDef bd;

        const auto x = RandomFloat(-10.0f, 10.0f);
        const auto y = RandomFloat(0.0f, 20.0f);
        bd.position = Vec2(x, y) * Meter;
        bd.angle = Radian * RandomFloat(-Pi, Pi);

        m_userData[m_bodyIndex] = index;
        bd.userData = m_userData + m_bodyIndex;

        if (index == 4)
        {
            bd.angularDamping = Real(0.02f) * Hertz;
        }

        m_bodies[m_bodyIndex] = m_world->CreateBody(bd);

        if (index < 4)
        {
            m_bodies[m_bodyIndex]->CreateFixture(m_polygons[index]);
        }
        else if (index < 5)
        {
            m_bodies[m_bodyIndex]->CreateFixture(m_circle);
        }
        else
        {
            m_bodies[m_bodyIndex]->CreateFixture(m_edge);
        }

        m_bodyIndex = GetModuloNext(m_bodyIndex, static_cast<decltype(m_bodyIndex)>(e_maxBodies));
    }

    void Destroy()
    {
        for (auto i = 0; i < e_maxBodies; ++i)
        {
            if (m_bodies[i])
            {
                m_world->Destroy(m_bodies[i]);
                m_bodies[i] = nullptr;
                return;
            }
        }
    }

    void KeyboardDown(Key key) override
    {
        switch (key)
        {
        case Key_1:
        case Key_2:
        case Key_3:
        case Key_4:
        case Key_5:
        case Key_6:
            Create(key - Key_1);
            break;

        case Key_D:
            Destroy();
            break;

        case Key_M:
                if (m_mode == Mode::e_closest)
            {
                m_mode = Mode::e_any;
            }
            else if (m_mode == Mode::e_any)
            {
                m_mode = Mode::e_multiple;
            }
            else if (m_mode == Mode::e_multiple)
            {
                m_mode = Mode::e_closest;
            }
            break;

        default:
            break;
        }
    }

    static const char* GetModeName(Mode mode) noexcept
    {
        switch (mode)
        {
            case Mode::e_closest: return "closest - find closest fixture along the ray";
            case Mode::e_any: return "any - check for obstruction";
            case Mode::e_multiple: return "multiple - gather multiple fixtures";
        }
        return "unknown";
    }

    void PostStep(const Settings& settings, Drawer& drawer) override
    {
        drawer.DrawString(5, m_textLine,
                          "Press '1' to drop triangles that should be ignored by the ray.");
        m_textLine += DRAW_STRING_NEW_LINE;
        drawer.DrawString(5, m_textLine,
                          "Press '2'-'6' to drop shapes that should not be ignored by the ray.");
        m_textLine += DRAW_STRING_NEW_LINE;

        drawer.DrawString(5, m_textLine,
                          "Press 'm' to change the mode of the raycast test (currently: %s).",
                          GetModeName(m_mode));
        m_textLine += DRAW_STRING_NEW_LINE;

        const auto L = 11.0f;
        const auto point1 = Vec2(0.0f, 10.0f) * Meter;
        const auto d = Vec2(L * std::cos(m_angle), L * std::sin(m_angle)) * Meter;
        const auto point2 = point1 + d;

        if (m_mode == Mode::e_closest)
        {
            auto hit = false;
            Length2D point;
            UnitVec2 normal;

            m_world->RayCast(point1, point2, [&](Fixture* f, const ChildCounter,
                                                 const Length2D& p, const UnitVec2& n)
            {
                const auto body = f->GetBody();
                const auto userData = body->GetUserData();
                if (userData)
                {
                    const auto index = *static_cast<int*>(userData);
                    if (index == 0)
                    {
                        // Instruct the calling code to ignore this fixture and
                        // continue the ray-cast to the next fixture.
                        return World::RayCastOpcode::IgnoreFixture;
                    }
                }

                hit = true;
                point = p;
                normal = n;
                
                // Instruct the calling code to clip the ray and
                // continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
                // are reported in order. However, by clipping, we can always get the closest fixture.
                return World::RayCastOpcode::ClipRay;
            });

            if (hit)
            {
                drawer.DrawPoint(point, 5.0f, Color(0.4f, 0.9f, 0.4f));
                drawer.DrawSegment(point1, point, Color(0.8f, 0.8f, 0.8f));
                const auto head = point + Real{0.5f} * normal * Meter;
                drawer.DrawSegment(point, head, Color(0.9f, 0.9f, 0.4f));
            }
            else
            {
                drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
            }
        }
        else if (m_mode == Mode::e_any)
        {
            auto hit = false;
            Length2D point;
            UnitVec2 normal;

            // This callback finds any hit. Polygon 0 is filtered. For this type of query we are
            // just checking for obstruction, so the actual fixture and hit point are irrelevant.
            m_world->RayCast(point1, point2, [&](Fixture* f, const ChildCounter,
                                                 const Length2D& p, const UnitVec2& n)
            {
                const auto body = f->GetBody();
                const auto userData = body->GetUserData();
                if (userData)
                {
                    const auto index = *static_cast<int*>(userData);
                    if (index == 0)
                    {
                        // Instruct the calling code to ignore this fixture
                        // and continue the ray-cast to the next fixture.
                        return World::RayCastOpcode::IgnoreFixture;
                    }
                }
                
                hit = true;
                point = p;
                normal = n;
                
                // At this point we have a hit, so we know the ray is obstructed.
                // Instruct the calling code to terminate the ray-cast.
                return World::RayCastOpcode::Terminate;
            });

            if (hit)
            {
                drawer.DrawPoint(point, 5.0f, Color(0.4f, 0.9f, 0.4f));
                drawer.DrawSegment(point1, point, Color(0.8f, 0.8f, 0.8f));
                const auto head = point + Real{0.5f} * normal * Meter;
                drawer.DrawSegment(point, head, Color(0.9f, 0.9f, 0.4f));
            }
            else
            {
                drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));
            }
        }
        else if (m_mode == Mode::e_multiple)
        {
            drawer.DrawSegment(point1, point2, Color(0.8f, 0.8f, 0.8f));

            // This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
            // The fixtures are not necessary reported in order, so we might not capture
            // the closest fixture.
            m_world->RayCast(point1, point2, [&](Fixture* f, const ChildCounter,
                                                 const Length2D& p, const UnitVec2& n)
            {
                const auto body = f->GetBody();
                const auto userData = body->GetUserData();
                if (userData)
                {
                    const auto index = *static_cast<int*>(userData);
                    if (index == 0)
                    {
                        // Instruct the calling code to ignore this fixture
                        // and continue the ray-cast to the next fixture.
                        return World::RayCastOpcode::IgnoreFixture;
                    }
                }
                
                drawer.DrawPoint(p, 5.0f, Color(0.4f, 0.9f, 0.4f));
                drawer.DrawSegment(point1, p, Color(0.8f, 0.8f, 0.8f));
                const auto head = p + Real{0.5f} * n * Meter;
                drawer.DrawSegment(p, head, Color(0.9f, 0.9f, 0.4f));
                
                // Instruct the caller to continue without clipping the ray.
                return World::RayCastOpcode::ResetRay;
            });
        }

        const auto advanceRay = !settings.pause || settings.singleStep;
        if (advanceRay)
        {
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

            PolygonShape shape;
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
    Body* m_bodies[e_maxBodies];
    int m_userData[e_maxBodies];
    std::shared_ptr<PolygonShape> m_polygons[4];
    std::shared_ptr<DiskShape> m_circle = std::make_shared<DiskShape>();
    std::shared_ptr<EdgeShape> m_edge = std::make_shared<EdgeShape>(Vec2(-1.0f, 0.0f) * Meter, Vec2(1.0f, 0.0f) * Meter);
    Real m_angle = 0.0f;
    Mode m_mode = Mode::e_closest;
};

} // namespace playrho

#endif
