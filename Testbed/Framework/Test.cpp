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

#include "Test.hpp"
#include "Drawer.hpp"
#include <stdio.h>
#include <vector>
#include <sstream>
#include <chrono>

#include <PlayRho/Rope/Rope.hpp>
#include <PlayRho/Dynamics/FixtureProxy.hpp>

using namespace playrho;

static void DrawCorner(Drawer& drawer, Length2D p, Length r, Angle a0, Angle a1, Color color)
{
    const auto angleDiff = GetRevRotationalAngle(a0, a1);
    auto lastAngle = Angle{0};
    for (auto angle = Degree * Real{5}; angle < angleDiff; angle += Degree * Real{5})
    {
        const auto c0 = p + r * UnitVec2::Get(a0 + lastAngle);
        const auto c1 = p + r * UnitVec2::Get(a0 + angle);
        drawer.DrawSegment(c0, c1, color);
        lastAngle = angle;
    }
    {
        const auto c0 = p + r * UnitVec2::Get(a0 + lastAngle);
        const auto c1 = p + r * UnitVec2::Get(a1);
        drawer.DrawSegment(c0, c1, color);
    }
}

class ShapeDrawer: public Shape::Visitor
{
public:
    ShapeDrawer(Drawer& d, Color c, bool s, Transformation t):
        drawer{d}, color{c}, skins{s}, xf{t}
    {
        // Intentionally empty.
    }
    
    void Visit(const DiskShape& shape) override;
    void Visit(const EdgeShape& shape) override;
    void Visit(const PolygonShape& shape) override;
    void Visit(const ChainShape& shape) override;
    void Visit(const MultiShape& shape) override;
    
    void Draw(const DistanceProxy& proxy);

    Drawer& drawer;
    Color color;
    bool skins;
    Transformation xf;
};

void ShapeDrawer::Visit(const DiskShape& shape)
{
    const auto center = Transform(shape.GetLocation(), xf);
    const auto radius = shape.GetRadius();
    const auto fillColor = Color{0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};
    drawer.DrawSolidCircle(center, radius, fillColor);
    drawer.DrawCircle(center, radius, color);

    // Draw a line fixed in the circle to animate rotation.
    const auto axis = Rotate(Vec2{Real{1}, Real{0}}, xf.q);
    drawer.DrawSegment(center, center + radius * axis, color);
}

void ShapeDrawer::Visit(const EdgeShape& shape)
{
    const auto v1 = Transform(shape.GetVertex1(), xf);
    const auto v2 = Transform(shape.GetVertex2(), xf);
    drawer.DrawSegment(v1, v2, color);
    
    if (skins)
    {
        const auto r = shape.GetVertexRadius();
        if (r > Length{0})
        {
            const auto skinColor = Color{color.r * 0.6f, color.g * 0.6f, color.b * 0.6f};
            const auto worldNormal0 = GetFwdPerpendicular(GetUnitVector(v2 - v1));
            const auto offset = worldNormal0 * r;
            drawer.DrawSegment(v1 + offset, v2 + offset, skinColor);
            drawer.DrawSegment(v1 - offset, v2 - offset, skinColor);
            
            const auto angle0 = GetAngle(worldNormal0);
            const auto angle1 = GetAngle(-worldNormal0);
            DrawCorner(drawer, v2, r, angle0, angle1, skinColor);
            DrawCorner(drawer, v1, r, angle1, angle0, skinColor);
        }
    }
}

void ShapeDrawer::Visit(const ChainShape& shape)
{
    const auto count = shape.GetVertexCount();
    const auto r = shape.GetVertexRadius();
    const auto skinColor = Color{color.r * 0.6f, color.g * 0.6f, color.b * 0.6f};
    
    auto v1 = Transform(shape.GetVertex(0), xf);
    for (auto i = decltype(count){1}; i < count; ++i)
    {
        const auto v2 = Transform(shape.GetVertex(i), xf);
        drawer.DrawSegment(v1, v2, color);
        if (skins && r > Length{0})
        {
            const auto worldNormal0 = GetFwdPerpendicular(GetUnitVector(v2 - v1));
            const auto offset = worldNormal0 * r;
            drawer.DrawSegment(v1 + offset, v2 + offset, skinColor);
            drawer.DrawSegment(v1 - offset, v2 - offset, skinColor);
            const auto angle0 = GetAngle(worldNormal0);
            const auto angle1 = GetAngle(-worldNormal0);
            DrawCorner(drawer, v2, r, angle0, angle1, skinColor);
            DrawCorner(drawer, v1, r, angle1, angle0, skinColor);
        }
        v1 = v2;
    }
}

void ShapeDrawer::Draw(const DistanceProxy& shape)
{
    const auto vertexCount = shape.GetVertexCount();
    auto vertices = std::vector<Length2D>(vertexCount);
    for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
    {
        vertices[i] = Transform(shape.GetVertex(i), xf);
    }
    const auto fillColor = Color{0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};
    drawer.DrawSolidPolygon(&vertices[0], vertexCount, fillColor);
    drawer.DrawPolygon(&vertices[0], vertexCount, color);
    
    if (!skins)
    {
        return;
    }

    const auto skinColor = Color{color.r * 0.6f, color.g * 0.6f, color.b * 0.6f};
    const auto r = shape.GetVertexRadius();
    for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i)
    {
        if (i > 0)
        {
            const auto worldNormal0 = Rotate(shape.GetNormal(i - 1), xf.q);
            const auto p0 = vertices[i-1] + worldNormal0 * r;
            const auto p1 = vertices[i] + worldNormal0 * r;
            drawer.DrawSegment(p0, p1, skinColor);
            const auto normal1 = shape.GetNormal(i);
            const auto worldNormal1 = Rotate(normal1, xf.q);
            const auto angle0 = GetAngle(worldNormal0);
            const auto angle1 = GetAngle(worldNormal1);
            DrawCorner(drawer, vertices[i], r, angle0, angle1, skinColor);
        }
    }
    if (vertexCount > 1)
    {
        const auto worldNormal0 = Rotate(shape.GetNormal(vertexCount - 1), xf.q);
        drawer.DrawSegment(vertices[vertexCount - 1] + worldNormal0 * r, vertices[0] + worldNormal0 * r, skinColor);
        const auto worldNormal1 = Rotate(shape.GetNormal(0), xf.q);
        const auto angle0 = GetAngle(worldNormal0);
        const auto angle1 = GetAngle(worldNormal1);
        DrawCorner(drawer, vertices[0], r, angle0, angle1, skinColor);
    }
    else if (vertexCount == 1)
    {
        DrawCorner(drawer, vertices[0], r, Angle{0}, Real{360} * Degree, skinColor);
    }
}

void ShapeDrawer::Visit(const PolygonShape& shape)
{
    Draw(shape.GetChild(0));
}

void ShapeDrawer::Visit(const MultiShape& shape)
{
    const auto count = shape.GetChildCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        Draw(shape.GetChild(i));
    }
}

static void Draw(Drawer& drawer, const Fixture& fixture, const Color& color, bool skins)
{
    const auto xf = GetTransformation(fixture);
    auto shapeDrawer = ShapeDrawer{drawer, color, skins, xf};
    const auto shape = fixture.GetShape();
    shape->Accept(shapeDrawer);
}

static Color GetColor(const Body& body)
{
    if (!body.IsEnabled())
    {
        return Color{0.5f, 0.5f, 0.3f};
    }
    if (body.GetType() == BodyType::Static)
    {
        return Color{0.5f, 0.9f, 0.5f};
    }
    if (body.GetType() == BodyType::Kinematic)
    {
        return Color{0.5f, 0.5f, 0.9f};
    }
    if (!body.IsAwake())
    {
        return Color{0.6f, 0.6f, 0.6f};
    }
    return Color{0.9f, 0.7f, 0.7f};
}

static bool Draw(Drawer& drawer, const Body& body, bool skins, Fixture* selected)
{
    auto found = false;
    const auto bodyColor = GetColor(body);
    const auto selectedColor = Brighten(bodyColor, 1.3f);
    for (auto&& fixture: body.GetFixtures())
    {
        const auto& f = GetRef(fixture);
        auto color = bodyColor;
        if (&f == selected)
        {
            color = selectedColor;
            found = true;
        }
        Draw(drawer, f, color, skins);
    }
    return found;
}

static void Draw(Drawer& drawer, const Joint& joint)
{
    const auto bodyA = joint.GetBodyA();
    const auto bodyB = joint.GetBodyB();
    const auto xf1 = bodyA->GetTransformation();
    const auto xf2 = bodyB->GetTransformation();
    const auto x1 = xf1.p;
    const auto x2 = xf2.p;
    const auto p1 = joint.GetAnchorA();
    const auto p2 = joint.GetAnchorB();
    
    const Color color{0.5f, 0.8f, 0.8f};
    
    switch (joint.GetType())
    {
        case JointType::Distance:
            drawer.DrawSegment(p1, p2, color);
            break;
            
        case JointType::Pulley:
        {
            const auto pulley = static_cast<const PulleyJoint&>(joint);
            const auto s1 = pulley.GetGroundAnchorA();
            const auto s2 = pulley.GetGroundAnchorB();
            drawer.DrawSegment(s1, p1, color);
            drawer.DrawSegment(s2, p2, color);
            drawer.DrawSegment(s1, s2, color);
        }
            break;
            
        case JointType::Mouse:
            // don't draw this
            break;
            
        default:
            drawer.DrawSegment(x1, p1, color);
            drawer.DrawSegment(p1, p2, color);
            drawer.DrawSegment(x2, p2, color);
    }
}

static bool Draw(Drawer& drawer, const World& world, const Settings& settings, Fixture* selected)
{
    auto found = false;

    if (settings.drawShapes)
    {
        for (auto&& body: world.GetBodies())
        {
            const auto b = GetPtr(body);
            if (Draw(drawer, *b, settings.drawSkins, selected))
            {
                found = true;
            }
        }
    }
    
    if (settings.drawJoints)
    {
        for (auto&& j: world.GetJoints())
        {
            Draw(drawer, *j);
        }
    }
    
    if (settings.drawAABBs)
    {
        const auto color = Color{0.9f, 0.3f, 0.9f};
        
        for (auto&& body: world.GetBodies())
        {
            const auto b = GetPtr(body);
            if (!b->IsEnabled())
            {
                continue;
            }
            
            for (auto&& fixture: b->GetFixtures())
            {
                const auto& f = GetRef(fixture);
                const auto proxy_count = f.GetProxyCount();
                for (auto i = decltype(proxy_count){0}; i < proxy_count; ++i)
                {
                    const auto proxy = f.GetProxy(i);
                    const auto aabb = world.GetFatAABB(proxy->proxyId);
                    Length2D vs[4];
                    vs[0] = Length2D{GetX(aabb.GetLowerBound()), GetY(aabb.GetLowerBound())};
                    vs[1] = Length2D{GetX(aabb.GetUpperBound()), GetY(aabb.GetLowerBound())};
                    vs[2] = Length2D{GetX(aabb.GetUpperBound()), GetY(aabb.GetUpperBound())};
                    vs[3] = Length2D{GetX(aabb.GetLowerBound()), GetY(aabb.GetUpperBound())};
                    
                    drawer.DrawPolygon(vs, 4, color);
                }
            }
        }
    }
    
    if (settings.drawCOMs)
    {
        const auto k_axisScale = Real(0.4) * Meter;
        const auto red = Color{1.0f, 0.0f, 0.0f};
        const auto green = Color{0.0f, 1.0f, 0.0f};
        for (auto&& body: world.GetBodies())
        {
            const auto b = GetPtr(body);
            auto xf = b->GetTransformation();
            xf.p = b->GetWorldCenter();
            const auto p1 = xf.p;
            drawer.DrawSegment(p1, p1 + k_axisScale * GetXAxis(xf.q), red);
            drawer.DrawSegment(p1, p1 + k_axisScale * GetYAxis(xf.q), green);            
        }
    }

    return found;
}

void Test::DestructionListenerImpl::SayGoodbye(Joint& joint)
{
    if (test->m_mouseJoint == &joint)
    {
        test->m_mouseJoint = nullptr;
    }
    else
    {
        test->JointDestroyed(&joint);
    }
}

Test::Test(const WorldDef& conf):
    m_world{new World(conf)}
{
    m_destructionListener.test = this;
    m_world->SetDestructionListener(&m_destructionListener);
    m_world->SetContactListener(this);
    
    m_groundBody = m_world->CreateBody();
}

Test::~Test()
{
    // By deleting the world, we delete the bomb, mouse joint, etc.
    delete m_world;
}

void Test::ResetWorld(const playrho::World &saved)
{
    SetSelectedFixture(nullptr);

    auto bombIndex = static_cast<decltype(m_world->GetBodies().size())>(-1);
    auto groundIndex = static_cast<decltype(m_world->GetBodies().size())>(-1);
    
    {
        auto i = decltype(m_world->GetBodies().size()){0};
        for (auto&& b: m_world->GetBodies())
        {
            const auto body = GetPtr(b);
            if (body == m_bomb)
            {
                bombIndex = i;
            }
            if (body == m_groundBody)
            {
                groundIndex = i;
            }
            ++i;
        }
    }

    *m_world = saved;
    
    {
        auto i = decltype(m_world->GetBodies().size()){0};
        for (auto&& b: m_world->GetBodies())
        {
            const auto body = GetPtr(b);
            if (i == bombIndex)
            {
                m_bomb = body;
            }
            if (i == groundIndex)
            {
                m_groundBody = body;
            }
            ++i;
        }
    }
}

void Test::PreSolve(Contact& contact, const Manifold& oldManifold)
{
    const auto pointStates = GetPointStates(oldManifold, contact.GetManifold());
    const auto worldManifold = GetWorldManifold(contact);
    
    ContactPoint cp;
    cp.fixtureA = contact.GetFixtureA();
    cp.fixtureB = contact.GetFixtureB();
    cp.normal = worldManifold.GetNormal();

    const auto count = worldManifold.GetPointCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto ci = worldManifold.GetImpulses(i);
        cp.normalImpulse = ci.m_normal;
        cp.tangentImpulse = ci.m_tangent;
        cp.state = pointStates.state2[i];
        cp.position = worldManifold.GetPoint(i);
        cp.separation = worldManifold.GetSeparation(i);
        m_points.push_back(cp);
    }
}

void Test::DrawTitle(Drawer& drawer, const char *string)
{
    drawer.DrawString(5, DRAW_STRING_NEW_LINE, string);
    m_textLine = 3 * DRAW_STRING_NEW_LINE;
}

void Test::MouseDown(const Length2D& p)
{
    m_mouseWorld = p;
    
    if (m_mouseJoint)
    {
        return;
    }

    // Make a small box.
    const auto aabb = GetFattenedAABB(AABB{p}, Meter / Real{1000});
    
    auto fixture = static_cast<Fixture*>(nullptr);
    
    // Query the world for overlapping shapes.
    m_world->QueryAABB(aabb, [&](Fixture* f, const ChildCounter) {
        if (TestPoint(*f, p))
        {
            fixture = f;
            return false; // We are done, terminate the query.
        }
        return true; // Continue the query.
    });
    
    SetSelectedFixture(fixture);
    if (fixture)
    {
        const auto body = fixture->GetBody();
        if (body->GetType() == BodyType::Dynamic)
        {
            MouseJointDef md;
            md.bodyA = m_groundBody;
            md.bodyB = body;
            md.target = p;
            md.maxForce = Real(10000.0f) * GetMass(*body) * MeterPerSquareSecond;
            m_mouseJoint = static_cast<MouseJoint*>(m_world->CreateJoint(md));
            body->SetAwake();
        }
    }
}

void Test::SpawnBomb(const Length2D& worldPt)
{
    m_bombSpawnPoint = worldPt;
    m_bombSpawning = true;
}
    
void Test::CompleteBombSpawn(const Length2D& p)
{
    if (!m_bombSpawning)
    {
        return;
    }

    const auto relP = m_bombSpawnPoint - p;
    const auto vel = LinearVelocity2D{
        Real{30} * GetX(relP) / Second,
        Real{30} * GetY(relP) / Second
    };
    LaunchBomb(m_bombSpawnPoint, vel);
    m_bombSpawning = false;
}

void Test::ShiftMouseDown(const Length2D& p)
{
    m_mouseWorld = p;
    
    if (m_mouseJoint)
    {
        return;
    }

    SpawnBomb(p);
}

void Test::MouseUp(const Length2D& p)
{
    if (m_mouseJoint)
    {
        m_world->Destroy(m_mouseJoint);
        m_mouseJoint = nullptr;
    }
    
    if (m_bombSpawning)
    {
        CompleteBombSpawn(p);
    }
}

void Test::MouseMove(const Length2D& p)
{
    m_mouseWorld = p;
    
    if (m_mouseJoint)
    {
        m_mouseJoint->SetTarget(p);
    }
}

void Test::LaunchBomb()
{
    const auto p = Length2D(RandomFloat(-15.0f, 15.0f) * Meter, Real(40.0f) * Meter);
    const auto v = LinearVelocity2D{
        Real{-100} * GetX(p) / Second,
        Real{-100} * GetY(p) / Second
    };
    LaunchBomb(p, v);
}

void Test::LaunchBomb(const Length2D& position, const LinearVelocity2D linearVelocity)
{
    if (m_bomb)
    {
        m_world->Destroy(m_bomb);
        m_bomb = nullptr;
    }

    m_bomb = m_world->CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(position).UseBullet(true));
    m_bomb->SetVelocity(Velocity{linearVelocity, AngularVelocity{0}});
    
    auto conf = DiskShape::Conf{};
    conf.vertexRadius = Real{0.3f} * Meter;
    conf.density = Density{Real{20} * Kilogram / SquareMeter};
    conf.restitution = 0.0f;
    const auto circle = std::make_shared<DiskShape>(conf);

    m_bomb->CreateFixture(circle);
}

void Test::DrawStats(Drawer& drawer, const StepConf& stepConf)
{
    const auto bodyCount = GetBodyCount(*m_world);
    const auto awakeCount = GetAwakeCount(*m_world);
    const auto sleepCount = bodyCount - awakeCount;
    const auto jointCount = GetJointCount(*m_world);
    const auto fixtureCount = GetFixtureCount(*m_world);
    const auto shapeCount = GetShapeCount(*m_world);
    const auto touchingCount = GetTouchingCount(*m_world);
    
    std::stringstream stream;
    
    drawer.DrawString(5, m_textLine, "step#=%d (@%fs):", m_stepCount, m_sumDeltaTime);
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "  Times:";
    stream << " cur=" << m_curStepDuration.count();
    stream << " max=" << m_maxStepDuration.count();
    stream << " sum=" << m_sumStepDuration.count();
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "  Object counts:";
    stream << " bodies=" << bodyCount << " (" << sleepCount << " asleep),";
    stream << " fixtures=" << fixtureCount << ",";
    stream << " shapes=" << shapeCount << ",";
    stream << " contacts=" << m_numContacts;
    stream << " (" << touchingCount << " touching, " << m_maxContacts << " max),";
    stream << " joints=" << jointCount;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;

    stream = std::stringstream();
    stream << "  pre-info:";
    stream << " cts-add=" << m_stepStats.pre.added;
    stream << " cts-ignor=" << m_stepStats.pre.ignored;
    stream << " cts-del=" << m_stepStats.pre.destroyed;
    stream << " cts-upd=" << m_stepStats.pre.updated;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    drawer.DrawString(5, m_textLine, "  reg-info:");
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "   ";
    stream << " cts-add=" << m_stepStats.reg.contactsAdded;
    stream << " isl-find=" << m_stepStats.reg.islandsFound;
    stream << " isl-solv=" << m_stepStats.reg.islandsSolved;
    stream << " pos-iter=" << m_stepStats.reg.sumPosIters;
    stream << " vel-iter=" << m_stepStats.reg.sumVelIters;
    stream << " proxy-moved=" << m_stepStats.toi.proxiesMoved;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;

    stream = std::stringstream();
    stream << "   ";
    stream << " bod-slept=" << m_stepStats.reg.bodiesSlept;
    stream << " min-sep=" << static_cast<double>(Real{m_stepStats.reg.minSeparation / Meter});
    stream << " max-inc-imp=" << static_cast<double>(Real{m_stepStats.reg.maxIncImpulse / (Kilogram * MeterPerSecond)});
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    drawer.DrawString(5, m_textLine, "  toi-info:");
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "   ";
    stream << " cts-add=" << m_stepStats.toi.contactsAdded;
    stream << " isl-find=" << m_stepStats.toi.islandsFound;
    stream << " isl-solv=" << m_stepStats.toi.islandsSolved;
    stream << " pos-iter=" << m_stepStats.toi.sumPosIters;
    stream << " vel-iter=" << m_stepStats.toi.sumVelIters;
    stream << " proxy-moved=" << m_stepStats.toi.proxiesMoved;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;

    stream = std::stringstream();
    stream << "   ";
    stream << " cts-find=" << m_stepStats.toi.contactsFound;
    stream << " cts-atmaxsubs=" << m_stepStats.toi.contactsAtMaxSubSteps;
    stream << " cts-upd=" << m_stepStats.toi.contactsUpdatedToi;
    stream << " max-dist-iter=" << unsigned{m_stepStats.toi.maxDistIters};
    stream << " max-toi-iter=" << unsigned{m_stepStats.toi.maxToiIters};
    stream << " min-sep=" << static_cast<double>(Real{m_stepStats.toi.minSeparation / Meter});
    stream << " max-inc-imp=" << static_cast<double>(Real{m_stepStats.toi.maxIncImpulse / (Kilogram * MeterPerSecond)});
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "  Pre sums:";
    // stream << " cts-ignored=" << m_sumContactsIgnoredPre;
    stream << " cts-upd=" << m_sumContactsUpdatedPre;
    stream << " cts-skipped=" << m_sumContactsSkippedPre;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "  Reg sums:";
    stream << " isl-found=" << m_sumRegIslandsFound;
    stream << " isl-solv=" << m_sumRegIslandsSolved;
    stream << " pos-iter=" << m_sumRegPosIters;
    stream << " vel-iter=" << m_sumRegVelIters;
    stream << " proxy-moved=" << m_sumRegProxiesMoved;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "  TOI sums:";
    stream << " isl-found=" << m_sumToiIslandsFound;
    stream << " isl-solv=" << m_sumToiIslandsSolved;
    stream << " pos-iter=" << m_sumToiPosIters;
    stream << " vel-iter=" << m_sumToiVelIters;
    stream << " proxy-moved=" << m_sumToiProxiesMoved;
    stream << " cts-touch-upd=" << m_sumToiContactsUpdatedTouching;
    stream << " cts-touch-skipped=" << m_sumToiContactsSkippedTouching;
    stream << " cts-upd-toi=" << m_sumContactsUpdatedToi;
    stream << " cts-maxstep=" << m_sumContactsAtMaxSubSteps;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "  Reg ranges:";
    stream << " min-sep=" << static_cast<double>(Real{m_minRegSep / Meter});
    stream << " max-sep=" << static_cast<double>(Real{m_maxRegSep / Meter});
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    stream = std::stringstream();
    stream << "  TOI ranges:";
    stream << " min-sep=" << static_cast<double>(Real{m_minToiSep / Meter});
    stream << " max-dist-iter=" << unsigned{m_maxDistIters} << "/" << unsigned{stepConf.maxDistanceIters};
    stream << " max-toi-iter=" << unsigned{m_maxToiIters} << "/" << unsigned{stepConf.maxToiIters};
    stream << " max-root-iter=" << unsigned{m_maxRootIters} << "/" << unsigned{stepConf.maxToiRootIters};
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
    
    const auto proxyCount = m_world->GetProxyCount();
    const auto height = m_world->GetTreeHeight();
    const auto balance = m_world->GetTreeBalance();
    const auto quality = m_world->GetTreeQuality();
    drawer.DrawString(5, m_textLine, "  proxies/height/balance/quality = %d/%d/%d/%g",
                      proxyCount, height, balance, quality);
    m_textLine += DRAW_STRING_NEW_LINE;
    
    const auto selectedFixture = GetSelectedFixture();
    if (selectedFixture)
    {
        DrawStats(drawer, *selectedFixture);
    }
}

void Test::DrawStats(Drawer& drawer, const Fixture& fixture)
{
    const auto density = fixture.GetDensity();
    const auto friction = fixture.GetFriction();
    const auto restitution = fixture.GetRestitution();
    const auto shape = fixture.GetShape();
    const auto childCount = shape->GetChildCount();
    const auto vertexRadius = shape->GetVertexRadius();
    const auto body = fixture.GetBody();
    const auto location = body->GetLocation();
    const auto angle = body->GetAngle();
    const auto velocity = body->GetVelocity();
    const auto contacts = body->GetContacts();
    
    auto numContacts = 0;
    auto numTouching = 0;
    auto numImpulses = 0;
    for (auto&& ci: contacts)
    {
        ++numContacts;
        const auto contact = GetContactPtr(ci);
        if (contact->IsTouching())
        {
            ++numTouching;
            const auto manifold = contact->GetManifold();
            numImpulses += manifold.GetPointCount();
        }
    }

    std::stringstream stream;
    stream << "Selected fixture:";
    stream << " pos={{";
    stream << static_cast<double>(Real{GetX(location) / Meter});
    stream << ",";
    stream << static_cast<double>(Real{GetY(location) / Meter});
    stream << "}, ";
    stream << static_cast<double>(Real{angle / Degree});
    stream << "}, ";
    stream << " vel={{";
    stream << static_cast<double>(Real{GetX(velocity.linear) / MeterPerSecond});
    stream << ",";
    stream << static_cast<double>(Real{GetY(velocity.linear) / MeterPerSecond});
    stream << "}, ";
    stream << static_cast<double>(Real{velocity.angular / DegreePerSecond});
    stream << "}";
    stream << " density=" << static_cast<double>(Real{density * SquareMeter / Kilogram});
    stream << " vr=" << static_cast<double>(Real{vertexRadius / Meter});
    stream << " childCount=" << std::size_t(childCount);
    stream << " friction=" << friction;
    stream << " restitution=" << restitution;
    stream << " b-cts=" << numTouching;
    stream << "/" << numContacts;
    stream << " b-impulses=" << numImpulses;
    drawer.DrawString(5, m_textLine, stream.str().c_str());
    m_textLine += DRAW_STRING_NEW_LINE;
}

void Test::DrawContactPoints(const Settings& settings, Drawer& drawer)
{
    const auto k_impulseScale = Real(0.1) * Second / Kilogram;
    const auto k_axisScale = (Real(3) / Real(10)) * Meter;
    const auto addStateColor = Color{0.3f, 0.9f, 0.3f}; // greenish
    const auto persistStateColor = Color{0.3f, 0.3f, 0.9f}; // blueish
    const auto contactNormalColor = Color{0.7f, 0.7f, 0.7f}; // light gray
    const auto normalImpulseColor = Color{0.9f, 0.9f, 0.3f}; // yellowish
    const auto frictionImpulseColor = Color{0.9f, 0.9f, 0.3f}; // yellowish
    const auto lighten = 1.3f;
    const auto darken = 0.9f;

    const auto selectedFixture = GetSelectedFixture();

    for (auto& point: m_points)
    {
        const auto selected = HasFixture(point, selectedFixture);
        if (point.state == PointState::AddState)
        {
            drawer.DrawPoint(point.position, Real{7} * Meter,
                             Brighten(addStateColor, selected? lighten: darken));
        }
        else if (point.state == PointState::PersistState)
        {
            // Persist
            drawer.DrawPoint(point.position, Real{5} * Meter,
                             Brighten(persistStateColor, selected? lighten: darken));
        }
 
        if (settings.drawContactImpulse)
        {
            const auto p1 = point.position;
            const auto p2 = p1 + k_impulseScale * point.normalImpulse * point.normal;
            drawer.DrawSegment(p1, p2, Brighten(normalImpulseColor, selected? lighten: darken));
        }
        
        if (settings.drawFrictionImpulse)
        {
            const auto tangent = GetFwdPerpendicular(point.normal);
            const auto p1 = point.position;
            const auto p2 = p1 + k_impulseScale * point.tangentImpulse * tangent;
            drawer.DrawSegment(p1, p2, Brighten(frictionImpulseColor, selected? lighten: darken));
        }
        
        if (settings.drawContactNormals)
        {
            const auto p1 = point.position;
            const auto p2 = p1 + k_axisScale * point.normal;
            drawer.DrawSegment(p1, p2, Brighten(contactNormalColor, selected? lighten: darken));
        }
    }
}

void Test::Step(const Settings& settings, Drawer& drawer)
{
    PreStep(settings, drawer);

    if (settings.pause)
    {
        drawer.DrawString(5, m_textLine, "****PAUSED****");
        m_textLine += DRAW_STRING_NEW_LINE;
        
        if ((settings.dt == 0) && m_mouseJoint)
        {
            const auto bodyB = m_mouseJoint->GetBodyB();
            const auto anchorB = m_mouseJoint->GetAnchorB();
            const auto centerB = bodyB->GetLocation();
            const auto destB = m_mouseJoint->GetTarget();
            bodyB->SetTransform(destB - (anchorB - centerB), bodyB->GetAngle());
        }
    }

    if (settings.dt != 0)
    {
        // Resets point count for contact point accumalation.
        m_points.clear();
    }

    m_world->SetSubStepping(settings.enableSubStepping);

    auto stepConf = StepConf{};
    
    stepConf.SetTime(Second * Real{settings.dt});
    
    stepConf.regVelocityIterations = static_cast<StepConf::iteration_type>(settings.regVelocityIterations);
    stepConf.regPositionIterations = static_cast<StepConf::iteration_type>(settings.regPositionIterations);
    stepConf.toiVelocityIterations = static_cast<StepConf::iteration_type>(settings.toiVelocityIterations);
    stepConf.toiPositionIterations = static_cast<StepConf::iteration_type>(settings.toiPositionIterations);

    stepConf.maxSubSteps = static_cast<StepConf::iteration_type>(settings.maxSubSteps);
    
    stepConf.maxTranslation = static_cast<Real>(settings.maxTranslation) * Meter;
    stepConf.maxRotation = Real{settings.maxRotation} * Degree;
    
    stepConf.linearSlop = Real{settings.linearSlop} * Meter;
    stepConf.angularSlop = Real{settings.angularSlop} * Radian;
    stepConf.regMinSeparation = Real{settings.regMinSeparation} * Meter;
    stepConf.toiMinSeparation = Real{settings.toiMinSeparation} * Meter;
    stepConf.targetDepth = settings.linearSlop * Real{3} * Meter;
    stepConf.tolerance = (settings.linearSlop / Real{4}) * Meter;
    
    stepConf.maxLinearCorrection = Real{settings.maxLinearCorrection} * Meter;
    stepConf.maxAngularCorrection = Real{settings.maxAngularCorrection} * Degree;
    stepConf.regResolutionRate = settings.regPosResRate / 100.0f;
    stepConf.toiResolutionRate = settings.toiPosResRate / 100.0f;
    if (!settings.enableSleep)
    {
        stepConf.minStillTimeToSleep = Second * GetInvalid<Real>();
        Awaken(*m_world);
    }
    stepConf.doToi = settings.enableContinuous;
    stepConf.doWarmStart = settings.enableWarmStarting;

    const auto start = std::chrono::system_clock::now();
    const auto stepStats = m_world->Step(stepConf);
    const auto end = std::chrono::system_clock::now();
    
    m_sumContactsUpdatedPre += stepStats.pre.updated;
    m_sumContactsIgnoredPre += stepStats.pre.ignored;
    m_sumContactsSkippedPre += stepStats.pre.skipped;

    m_sumRegIslandsFound += stepStats.reg.islandsFound;
    m_sumRegIslandsSolved += stepStats.reg.islandsSolved;
    m_sumRegPosIters += stepStats.reg.sumPosIters;
    m_sumRegVelIters += stepStats.reg.sumVelIters;
    m_sumRegProxiesMoved += stepStats.reg.proxiesMoved;

    m_sumToiIslandsFound += stepStats.toi.islandsFound;
    m_sumToiIslandsSolved += stepStats.toi.islandsSolved;
    m_sumToiPosIters += stepStats.toi.sumPosIters;
    m_sumToiVelIters += stepStats.toi.sumVelIters;
    m_sumToiProxiesMoved += stepStats.toi.proxiesMoved;
    m_sumContactsUpdatedToi += stepStats.toi.contactsUpdatedToi;
    m_sumToiContactsUpdatedTouching += stepStats.toi.contactsUpdatedTouching;
    m_sumToiContactsSkippedTouching += stepStats.toi.contactsSkippedTouching;
    m_sumContactsAtMaxSubSteps += stepStats.toi.contactsAtMaxSubSteps;

    m_maxDistIters = std::max(m_maxDistIters, stepStats.toi.maxDistIters);
    m_maxRootIters = std::max(m_maxRootIters, stepStats.toi.maxRootIters);
    m_maxToiIters = std::max(m_maxToiIters, stepStats.toi.maxToiIters);

    if (stepStats.reg.minSeparation < std::numeric_limits<Real>::infinity() * Meter)
    {
        m_minRegSep = std::min(m_minRegSep, stepStats.reg.minSeparation);
        m_maxRegSep = std::max(m_maxRegSep, stepStats.reg.minSeparation);
    }
    
    if (settings.dt != 0)
    {
        m_sumDeltaTime += settings.dt;
        ++m_stepCount;
        m_stepStats = stepStats;
        m_minToiSep = std::min(m_minToiSep, stepStats.toi.minSeparation);
        
        m_curStepDuration = end - start;
        m_maxStepDuration = std::max(m_maxStepDuration, m_curStepDuration);
        m_sumStepDuration += m_curStepDuration;
    }

    m_numContacts = GetContactCount(*m_world);
    m_maxContacts = std::max(m_maxContacts, m_numContacts);

    if (settings.drawStats)
    {
        DrawStats(drawer, stepConf);
    }

    if (m_mouseJoint)
    {
        const auto p1 = m_mouseJoint->GetAnchorB();
        const auto p2 = m_mouseJoint->GetTarget();

        drawer.DrawPoint(p1, Real{4} * Meter, Color{0.0f, 1.0f, 0.0f});
        drawer.DrawPoint(p2, Real{4} * Meter, Color{0.0f, 1.0f, 0.0f});

        drawer.DrawSegment(p1, p2, Color{0.8f, 0.8f, 0.8f});
    }
    
    if (m_bombSpawning)
    {
        drawer.DrawPoint(m_bombSpawnPoint, Real{4} * Meter, Color{0.0f, 0.0f, 1.0f});
        drawer.DrawSegment(m_mouseWorld, m_bombSpawnPoint, Color{0.8f, 0.8f, 0.8f});
    }

    if (settings.drawContactPoints)
    {
        DrawContactPoints(settings, drawer);
    }
    
    PostStep(settings, drawer);
    
    const auto selectedFixture = GetSelectedFixture();
    const auto selectedFound = Draw(drawer, *m_world, settings, selectedFixture);
    if (selectedFixture && !selectedFound)
    {
        SetSelectedFixture(nullptr);
    }
    
    drawer.Flush();
}

void Test::ShiftOrigin(const Length2D& newOrigin)
{
    m_world->ShiftOrigin(newOrigin);
}

constexpr auto RAND_LIMIT = 32767;

Real playrho::RandomFloat()
{
    auto r = static_cast<Real>(std::rand() & (RAND_LIMIT));
    r /= RAND_LIMIT;
    r = 2.0f * r - 1.0f;
    return r;
}

Real playrho::RandomFloat(Real lo, Real hi)
{
    auto r = static_cast<Real>(std::rand() & (RAND_LIMIT));
    r /= RAND_LIMIT;
    r = (hi - lo) * r + lo;
    return r;
}

static const char* GetName(ContactFeature::Type type)
{
    switch (type)
    {
        case ContactFeature::e_face: return "face";
        case ContactFeature::e_vertex: return "vertex";
    }
    return "unknown";
}

::std::ostream& playrho::operator<<(::std::ostream& os, const ContactFeature& value)
{
    os << "{";
    os << ::GetName(value.typeA);
    os << ",";
    os << unsigned(value.indexA);
    os << ",";
    os << ::GetName(value.typeB);
    os << ",";
    os << unsigned(value.indexB);
    os << "}";
    return os;
}
