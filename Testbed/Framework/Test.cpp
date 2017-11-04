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
#include <utility>
#include "imgui.h"

using namespace playrho;

static void DrawCorner(Drawer& drawer, Length2 p, Length r, Angle a0, Angle a1, Color color)
{
    const auto angleDiff = GetRevRotationalAngle(a0, a1);
    auto lastAngle = 0_deg;
    for (auto angle = 5_deg; angle < angleDiff; angle += 5_deg)
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

class ShapeDrawer: public ShapeVisitor
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
    auto vertices = std::vector<Length2>(vertexCount);
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
        DrawCorner(drawer, vertices[0], r, Angle{0}, 360_deg, skinColor);
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
        return Color{0.75f, 0.75f, 0.75f};
    }
    return Color{0.9f, 0.7f, 0.7f};
}

static bool Draw(Drawer& drawer, const Body& body, bool skins, const Test::FixtureSet& selected)
{
    auto found = false;
    const auto bodyColor = GetColor(body);
    const auto selectedColor = Brighten(bodyColor, 1.3f);
    for (auto&& fixture: body.GetFixtures())
    {
        const auto& f = GetRef(fixture);
        auto color = bodyColor;
        if (Test::Contains(selected, &f))
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
    const auto p1 = joint.GetAnchorA();
    const auto p2 = joint.GetAnchorB();

    const Color color{0.5f, 0.8f, 0.8f};

    switch (GetType(joint))
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
        {
            const auto bodyA = joint.GetBodyA();
            const auto bodyB = joint.GetBodyB();
            const auto x1 = bodyA->GetTransformation().p;
            const auto x2 = bodyB->GetTransformation().p;
            drawer.DrawSegment(x1, p1, color);
            drawer.DrawSegment(p1, p2, color);
            drawer.DrawSegment(x2, p2, color);
        }
    }
}

static void Draw(Drawer& drawer, const AABB& aabb, const Color& color)
{
    Length2 vs[4];
    vs[0] = Length2{aabb.rangeX.GetMin(), aabb.rangeY.GetMin()};
    vs[1] = Length2{aabb.rangeX.GetMax(), aabb.rangeY.GetMin()};
    vs[2] = Length2{aabb.rangeX.GetMax(), aabb.rangeY.GetMax()};
    vs[3] = Length2{aabb.rangeX.GetMin(), aabb.rangeY.GetMax()};
    drawer.DrawPolygon(vs, 4, color);
}

bool Test::DrawWorld(Drawer& drawer, const World& world, const Settings& settings,
                     const FixtureSet& selected)
{
    auto found = false;

    if (settings.drawShapes)
    {
        const auto drawLabels = [&]() {
            const auto useField = m_neededSettings & (1u << NeedDrawLabelsField);
            return useField? m_settings.drawLabels: settings.drawLabels;
        }();
        const auto drawSkins = [&]() {
            const auto useField = m_neededSettings & (0x1u << NeedDrawSkinsField);
            return useField? m_settings.drawSkins: settings.drawSkins;
        }();
        
        for (auto&& body: world.GetBodies())
        {
            const auto b = GetPtr(body);
            if (Draw(drawer, *b, drawSkins, selected))
            {
                found = true;
            }
            if (drawLabels)
            {
                // Use center of mass instead of body center since body center may not
                drawer.DrawString(b->GetWorldCenter(), Drawer::Center, "%d", GetWorldIndex(b));
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
        const auto root = world.GetTree().GetRootIndex();
        if (root != DynamicTree::GetInvalidSize())
        {
            const auto worldAabb = world.GetTree().GetAABB(root);
            Draw(drawer, worldAabb, color);
            Query(world.GetTree(), worldAabb, [&](DynamicTree::Size id) {
                Draw(drawer, world.GetTree().GetAABB(id), color);
                return DynamicTreeOpcode::Continue;
            });
        }
    }

    if (settings.drawCOMs)
    {
        const auto k_axisScale = 0.4_m;
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

bool Test::Contains(const FixtureSet& fixtures, const Fixture* f) noexcept
{
    return fixtures.find(const_cast<Fixture*>(f)) != fixtures.end();
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

Test::Test(Conf conf):
    m_world(conf.worldDef),
    m_neededSettings(conf.neededSettings),
    m_settings(conf.settings),
    m_description(conf.description),
    m_credits(conf.credits),
    m_seeAlso(conf.seeAlso),
    m_numContactsPerStep(m_maxHistory, 0u),
    m_numTouchingPerStep(m_maxHistory, 0u)
{
    m_destructionListener.test = this;
    m_world.SetDestructionListener(&m_destructionListener);
    m_world.SetContactListener(this);
}

Test::~Test()
{
}

void Test::ResetWorld(const playrho::World &saved)
{
    ClearSelectedFixtures();

    auto bombIndex = static_cast<decltype(m_world.GetBodies().size())>(-1);

    {
        auto i = decltype(m_world.GetBodies().size()){0};
        for (auto&& b: m_world.GetBodies())
        {
            const auto body = GetPtr(b);
            if (body == m_bomb)
            {
                bombIndex = i;
            }
            ++i;
        }
    }

    m_world = saved;

    {
        auto i = decltype(m_world.GetBodies().size()){0};
        for (auto&& b: m_world.GetBodies())
        {
            const auto body = GetPtr(b);
            if (i == bombIndex)
            {
                m_bomb = body;
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
        cp.normalImpulse = Get<0>(ci);
        cp.tangentImpulse = Get<1>(ci);
        cp.state = pointStates.state2[i];
        cp.position = worldManifold.GetPoint(i);
        cp.separation = worldManifold.GetSeparation(i);
        m_points.push_back(cp);
    }
}

template <class T>
static std::set<Body*> GetBodySetFromFixtures(const T& fixtures)
{
    auto collection = std::set<Body*>();
    for (auto f: fixtures)
    {
        collection.insert(f->GetBody());
    }
    return collection;
}

void Test::SetSelectedFixtures(FixtureSet value) noexcept
{
    m_selectedFixtures = value;
    m_selectedBodies = GetBodySetFromFixtures(value);
}

void Test::MouseDown(const Length2& p)
{
    m_mouseWorld = p;

    if (m_mouseJoint)
    {
        return;
    }

    // Make a small box.
    const auto aabb = GetFattenedAABB(AABB{p}, 1_m / 1000);

    auto fixtures = FixtureSet{};

    // Query the world for overlapping shapes.
    m_world.QueryAABB(aabb, [&](Fixture* f, const ChildCounter) {
        if (TestPoint(*f, p))
        {
            fixtures.insert(f);
        }
        return true; // Continue the query.
    });

    SetSelectedFixtures(fixtures);
    if (fixtures.size() == 1)
    {
        const auto body = (*(fixtures.begin()))->GetBody();
        if (body->GetType() == BodyType::Dynamic)
        {
            auto md = MouseJointDef{};
            md.bodyB = body;
            md.target = p;
            md.maxForce = Real(10000.0f) * GetMass(*body) * MeterPerSquareSecond;
            m_mouseJoint = static_cast<MouseJoint*>(m_world.CreateJoint(md));
            body->SetAwake();
        }
    }
}

void Test::SpawnBomb(const Length2& worldPt)
{
    m_bombSpawnPoint = worldPt;
    m_bombSpawning = true;
}

void Test::CompleteBombSpawn(const Length2& p)
{
    if (!m_bombSpawning)
    {
        return;
    }

    const auto relP = m_bombSpawnPoint - p;
    const auto vel = LinearVelocity2{
        Real{30} * GetX(relP) / Second,
        Real{30} * GetY(relP) / Second
    };
    LaunchBomb(m_bombSpawnPoint, vel);
    m_bombSpawning = false;
}

void Test::ShiftMouseDown(const Length2& p)
{
    m_mouseWorld = p;

    if (m_mouseJoint)
    {
        return;
    }

    SpawnBomb(p);
}

void Test::MouseUp(const Length2& p)
{
    if (m_mouseJoint)
    {
        m_world.Destroy(m_mouseJoint);
        m_mouseJoint = nullptr;
    }

    if (m_bombSpawning)
    {
        CompleteBombSpawn(p);
    }
}

void Test::MouseMove(const Length2& p)
{
    m_mouseWorld = p;

    if (m_mouseJoint)
    {
        m_mouseJoint->SetTarget(p);
    }
}

void Test::LaunchBomb()
{
    const auto p = Length2(RandomFloat(-15.0f, 15.0f) * 1_m, 40_m);
    const auto v = LinearVelocity2{
        Real{-100} * GetX(p) / Second,
        Real{-100} * GetY(p) / Second
    };
    LaunchBomb(p, v);
}

void Test::LaunchBomb(const Length2& position, const LinearVelocity2 linearVelocity)
{
    if (m_bomb)
    {
        m_world.Destroy(m_bomb);
        m_bomb = nullptr;
    }

    m_bomb = m_world.CreateBody(BodyDef{}.UseType(BodyType::Dynamic).UseLocation(position).UseBullet(true));
    m_bomb->SetVelocity(Velocity{linearVelocity, AngularVelocity{0}});

    auto conf = DiskShape::Conf{};
    conf.vertexRadius = 0.3_m;
    conf.density = 20_kgpm2;
    conf.restitution = 0.0f;
    const auto circle = std::make_shared<DiskShape>(conf);

    m_bomb->CreateFixture(circle);
}

static void ShowHelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::ShowTooltip(desc, 400);
    }
}

void Test::DrawStats(const StepConf& stepConf, UiState& ui)
{
    const auto bodyCount = GetBodyCount(m_world);
    const auto awakeCount = GetAwakeCount(m_world);
    const auto sleepCount = bodyCount - awakeCount;
    const auto jointCount = GetJointCount(m_world);
    const auto fixtureCount = GetFixtureCount(m_world);
    const auto shapeCount = GetShapeCount(m_world);
    const auto touchingCount = GetTouchingCount(m_world);
 
    if (m_numTouchingPerStep.size() >= m_maxHistory)
    {
        m_numTouchingPerStep.pop_front();
    }
    m_numTouchingPerStep.push_back(touchingCount);
    m_maxTouching = std::max(m_maxTouching, touchingCount);

    ImGuiStyle& style = ImGui::GetStyle();
    const auto totalWidth = ImGui::GetWindowWidth() - style.FramePadding.x * 2;
    const auto firstColumnWidth = 65.0f;

    ImGui::Text("Step #=%d (@%fs):", m_stepCount, m_sumDeltaTime);
    ImGui::Separator();

    {
        ImGui::ColumnsContext cc(4, "TimesColumns", false);
        ImGui::SetColumnWidth(0, firstColumnWidth);
        ImGui::TextUnformatted("Times:");
        ImGui::NextColumn();
        ImGui::Value("Current", m_curStepDuration.count(), "%f");
        ImGui::NextColumn();
        ImGui::Value("Max", m_maxStepDuration.count(), "%f");
        ImGui::NextColumn();
        ImGui::Value("Sum", m_sumStepDuration.count(), "%f");
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(5, "NumObjectsColumns", false);
        ImGui::SetColumnWidth(0, firstColumnWidth);
        ImGui::TextUnformatted("# Objects:");
        ImGui::NextColumn();
        ImGui::Text("Bodies: %u/%u", bodyCount - sleepCount, bodyCount);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Counts of awake bodies over total bodies.");
        }
        ImGui::NextColumn();
        ImGui::Text("Fixtures: %lu/%lu", shapeCount, fixtureCount);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Counts of shapes over fixtures.");
        }
        ImGui::NextColumn();
        ImGui::Text("Contacts: %u/%u", touchingCount, m_numContacts);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Counts of touching contacts over total contacts. Click to toggle histogram.");
        }
        if (ImGui::IsItemClicked())
        {
            ui.showContactsHistory = !ui.showContactsHistory;
        }
        ImGui::NextColumn();
        ImGui::Value("Joints", jointCount);
        ImGui::NextColumn();
    }
    
    {
        ImGui::ColumnsContext cc(6, "PreStepColumns", false);
        ImGui::SetColumnWidth(0, firstColumnWidth);
        ImGui::TextUnformatted("Pre-step:");
        ImGui::NextColumn();
        ImGui::Value("cts-add", m_stepStats.pre.added);
        ImGui::NextColumn();
        ImGui::TextUnformatted([=]() {
            std::ostringstream os;
            os << "c-ign: " << m_stepStats.pre.ignored << "/" << m_sumContactsIgnoredPre;
            return os.str();
        }());
        ImGui::NextColumn();
        ImGui::TextUnformatted([=]() {
            std::ostringstream os;
            os << "c-skip: " << m_stepStats.pre.skipped << "/" << m_sumContactsSkippedPre;
            return os.str();
        }());
        ImGui::NextColumn();
        ImGui::Value("c-del", m_stepStats.pre.destroyed);
        ImGui::NextColumn();
        ImGui::TextUnformatted([=]() {
            std::ostringstream os;
            os << "c-upd: " << m_stepStats.pre.updated << "/" << m_sumContactsUpdatedPre;
            return os.str();
        }());
        ImGui::NextColumn();
    }

    ImGui::Separator();

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::NextColumn();
        ImGui::TextUnformatted("c-add");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Contacts added.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("i-find");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Islands found.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("i-solv");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Islands solved.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("posit");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Position iterations.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("velit");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Velocity iterations.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("pmov");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Proxies moved.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("minsep");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Minimum separation.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("maxP");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Max incremental impulse.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("b-slept");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Bodies slept.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("c-find");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Contacts found.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("c-@maxs");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Contacts at max substeps.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("c-upd");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Contacts whose TOI was updated.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("max-d-it");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Max distance iterations.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("max-t-it");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", "Max TOI iterations.");
        }
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("Reg. step:");
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.reg.contactsAdded);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.reg.islandsFound);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.reg.islandsSolved);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.reg.sumPosIters);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.reg.sumVelIters);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.proxiesMoved);
        ImGui::NextColumn();
        ImGui::Text("%f", static_cast<double>(Real{m_stepStats.reg.minSeparation / Meter}));
        ImGui::NextColumn();
        ImGui::Text("%.2f", static_cast<double>(Real{m_stepStats.reg.maxIncImpulse / NewtonSecond}));
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.reg.bodiesSlept);
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("TOI step:");
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.contactsAdded);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.islandsFound);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.islandsSolved);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.sumPosIters);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.sumVelIters);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.proxiesMoved);
        ImGui::NextColumn();
        ImGui::Text("%f", static_cast<double>(Real{m_stepStats.toi.minSeparation / Meter}));
        ImGui::NextColumn();
        ImGui::Text("%.2f", static_cast<double>(Real{m_stepStats.toi.maxIncImpulse / NewtonSecond}));
        ImGui::NextColumn();
        // Skip bodies slept column
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.contactsFound);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.contactsAtMaxSubSteps);
        ImGui::NextColumn();
        ImGui::Text("%u", m_stepStats.toi.contactsUpdatedToi);
        ImGui::NextColumn();
        ImGui::Text("%u", unsigned{m_stepStats.toi.maxDistIters});
        ImGui::NextColumn();
        ImGui::Text("%u", unsigned{m_stepStats.toi.maxToiIters});
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("Reg. sums:");
        ImGui::NextColumn();
        // Skip c-add column
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumRegIslandsFound));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumRegIslandsSolved));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumRegPosIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumRegVelIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumRegProxiesMoved));
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("TOI sums:");
        ImGui::NextColumn();
        // Skip c-add column
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumToiIslandsFound));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumToiIslandsSolved));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumToiPosIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumToiVelIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumToiProxiesMoved));
        ImGui::NextColumn();
        // Skip minSeparation column
        ImGui::NextColumn();
        // Skip maxIncImpulse column
        ImGui::NextColumn();
        // Skip bodies slept column
        ImGui::NextColumn();
        // Skip contacts found column
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumContactsAtMaxSubSteps));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(m_sumContactsUpdatedToi));
        ImGui::NextColumn();
        
#if 0
        stream = std::ostringstream();
        stream << "  TOI sums:";
        stream << " cts-touch-upd=" << m_sumToiContactsUpdatedTouching;
        stream << " cts-touch-skipped=" << m_sumToiContactsSkippedTouching;
#endif
    }

    ImGui::Separator();

    {
        ImGui::ColumnsContext cc(2, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("Reg ranges:");
        ImGui::NextColumn();
        std::ostringstream stream;
        stream << "min-sep=" << static_cast<double>(Real{m_minRegSep / Meter});
        stream << ", max-sep=" << static_cast<double>(Real{m_maxRegSep / Meter});
        stream << ".";
        ImGui::TextUnformatted(stream.str());
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(2, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("TOI ranges:");
        ImGui::NextColumn();
        std::ostringstream stream;
        stream << "min-sep=" << static_cast<double>(Real{m_minToiSep / Meter});
        stream << ", max-dist-iter=" << unsigned{m_maxDistIters} << "/" << unsigned{stepConf.maxDistanceIters};
        stream << ", max-toi-iter=" << unsigned{m_maxToiIters} << "/" << unsigned{stepConf.maxToiIters};
        stream << ", max-root-iter=" << unsigned{m_maxRootIters} << "/" << unsigned{stepConf.maxToiRootIters};
        stream << ".";
        ImGui::TextUnformatted(stream.str());
        ImGui::NextColumn();
    }

    {
        const auto leafCount = m_world.GetTree().GetLeafCount();
        const auto nodeCount = m_world.GetTree().GetNodeCount();
        const auto height = GetHeight(m_world.GetTree());
        const auto balance = m_world.GetTree().GetMaxBalance();
        const auto quality = ComputePerimeterRatio(m_world.GetTree());
        const auto capacity = m_world.GetTree().GetNodeCapacity();

        ImGui::ColumnsContext cc(2, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("Dyn. tree:");
        ImGui::NextColumn();
        ImGui::Text("nodes=%u/%u/%u, ", leafCount, nodeCount, capacity);
        ImGui::SameLine(0, 0);
        ImGui::Text("height=%u, ", height);
        ImGui::SameLine(0, 0);
        ImGui::Text("bal=%u, ", balance);
        ImGui::SameLine(0, 0);
        ImGui::Text("p-rat=%.2f, ", static_cast<double>(quality));
        ImGui::SameLine(0, 0);
        std::ostringstream stream;
        stream << m_maxAABB;
        ImGui::Text("max-aabb=%s.", stream.str().c_str());
        ImGui::NextColumn();
    }
}

void Test::DrawContactInfo(const Settings& settings, Drawer& drawer)
{
    const auto k_impulseScale = 0.1_s / 1_kg;
    const auto k_axisScale = 0.3_m;
    const auto addStateColor = Color{0.3f, 0.9f, 0.3f}; // greenish
    const auto persistStateColor = Color{0.3f, 0.3f, 0.9f}; // blueish
    const auto contactNormalColor = Color{0.7f, 0.7f, 0.7f}; // light gray
    const auto normalImpulseColor = Color{0.9f, 0.9f, 0.3f}; // yellowish
    const auto frictionImpulseColor = Color{0.9f, 0.9f, 0.3f}; // yellowish

    const auto selectedFixtures = GetSelectedFixtures();
    const auto lighten = 1.3f;
    const auto darken = 0.9f;

    for (auto& point: m_points)
    {
        const auto selected = HasFixture(point, selectedFixtures);

        if (settings.drawContactPoints)
        {
            if (point.state == PointState::AddState)
            {
                drawer.DrawPoint(point.position, 7.0f,
                                 Brighten(addStateColor, selected? lighten: darken));
            }
            else if (point.state == PointState::PersistState)
            {
                // Persist
                drawer.DrawPoint(point.position, 5.0f,
                                 Brighten(persistStateColor, selected? lighten: darken));
            }
        }

        if (settings.drawContactImpulse)
        {
            const auto length = k_impulseScale * point.normalImpulse;
            const auto headLength = length / Real(10);
            const auto p1 = point.position;
            const auto p2 = p1 + length * point.normal;
            const auto p2_left = p2 - headLength * Rotate(point.normal, UnitVec2::GetTopRight());
            const auto p2_right = p2 - headLength * Rotate(point.normal, UnitVec2::GetBottomRight());
            drawer.DrawSegment(p1, p2, Brighten(normalImpulseColor, selected? lighten: darken));
            drawer.DrawSegment(p2, p2_left, Brighten(normalImpulseColor, selected? lighten: darken));
            drawer.DrawSegment(p2, p2_right, Brighten(normalImpulseColor, selected? lighten: darken));
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

template <typename T>
struct DequeValuesGetter
{
    static float Func(void* data, int idx)
    {
        const std::deque<T>& deque = *static_cast<std::deque<T>*>(data);
        const auto size = deque.size();
        return (idx >= 0 && static_cast<decltype(size)>(idx) < size)?
            static_cast<float>(deque[static_cast<decltype(size)>(idx)]): 0.0f;
    }
};

void Test::Step(const Settings& settings, Drawer& drawer, UiState& ui)
{
    m_textLine = 3 * DRAW_STRING_NEW_LINE;

    PreStep(settings, drawer);

    if (settings.pause)
    {
        if ((settings.dt == 0) && m_mouseJoint)
        {
            const auto bodyB = m_mouseJoint->GetBodyB();
            const auto anchorB = m_mouseJoint->GetAnchorB();
            const auto centerB = bodyB->GetLocation();
            const auto destB = m_mouseJoint->GetTarget();
            //m_points.clear();
            bodyB->SetTransform(destB - (anchorB - centerB), bodyB->GetAngle());
        }
    }

    if (settings.dt != 0)
    {
        // Resets point count for contact point accumalation.
        m_points.clear();
    }

    m_world.SetSubStepping(settings.enableSubStepping);

    auto stepConf = StepConf{};

    stepConf.SetTime(settings.dt * Second);

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
    stepConf.targetDepth = 3 * settings.linearSlop * Meter;
    stepConf.tolerance = (settings.linearSlop / 4) * Meter;

    stepConf.maxLinearCorrection = Real{settings.maxLinearCorrection} * Meter;
    stepConf.maxAngularCorrection = Real{settings.maxAngularCorrection} * Degree;
    stepConf.regResolutionRate = settings.regPosResRate / 100.0f;
    stepConf.toiResolutionRate = settings.toiPosResRate / 100.0f;
    if (!settings.enableSleep)
    {
        stepConf.minStillTimeToSleep = std::numeric_limits<Time>::infinity();
        Awaken(m_world);
    }
    stepConf.doToi = settings.enableContinuous;
    stepConf.doWarmStart = settings.enableWarmStarting;

    const auto start = std::chrono::system_clock::now();
    const auto stepStats = m_world.Step(stepConf);
    const auto end = std::chrono::system_clock::now();

    m_maxAABB = GetEnclosingAABB(m_maxAABB, GetAABB(m_world.GetTree()));
    
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

    if (stepStats.reg.minSeparation < std::numeric_limits<Length>::infinity())
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

    m_numContacts = GetContactCount(m_world);
    m_maxContacts = std::max(m_maxContacts, m_numContacts);
    
    if (m_numContactsPerStep.size() >= m_maxHistory)
    {
        m_numContactsPerStep.pop_front();
    }
    m_numContactsPerStep.push_back(m_numContacts);

    if (ui.showStats)
    {
        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_Appearing);
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
        ImGui::WindowContext wc("Step Statistics", &ui.showStats, ImGuiWindowFlags_NoCollapse);
        DrawStats(stepConf, ui);
    }
    
    if (ui.showContactsHistory)
    {
        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_Once);
        ImGui::WindowContext wc("Contacts Histogram", &ui.showContactsHistory,
                                ImGuiWindowFlags_NoCollapse);
        char buffer[40];
        
        std::sprintf(buffer, "Max of %u", m_maxTouching);
        ImGui::PlotHistogram("# Touching", DequeValuesGetter<std::size_t>::Func,
                             &m_numTouchingPerStep, static_cast<int>(m_numTouchingPerStep.size()),
                             0, buffer, 0.0f, static_cast<float>(m_maxContacts),
                             ImVec2(600, 100));

        std::sprintf(buffer, "Max of %u", m_maxContacts);
        ImGui::PlotHistogram("# Contacts", DequeValuesGetter<std::size_t>::Func,
                             &m_numContactsPerStep, static_cast<int>(m_numContactsPerStep.size()),
                             0, buffer, 0.0f, static_cast<float>(m_maxContacts),
                             ImVec2(600, 100));
    }

    if (m_mouseJoint)
    {
        const auto p1 = m_mouseJoint->GetAnchorB();
        const auto p2 = m_mouseJoint->GetTarget();

        drawer.DrawPoint(p1, 4.0f, Color{0.0f, 1.0f, 0.0f});
        drawer.DrawPoint(p2, 4.0f, Color{0.0f, 1.0f, 0.0f});

        drawer.DrawSegment(p1, p2, Color{0.8f, 0.8f, 0.8f});
    }

    if (m_bombSpawning)
    {
        drawer.DrawPoint(m_bombSpawnPoint, 4.0f, Color{0.0f, 0.0f, 1.0f});
        drawer.DrawSegment(m_mouseWorld, m_bombSpawnPoint, Color{0.8f, 0.8f, 0.8f});
    }

    DrawContactInfo(settings, drawer);

    PostStep(settings, drawer);

    const auto selectedFixtures = GetSelectedFixtures();
    const auto selectedFound = DrawWorld(drawer, m_world, settings, selectedFixtures);
    if (!selectedFixtures.empty() && !selectedFound)
    {
        ClearSelectedFixtures();
    }

    drawer.Flush();
}

void Test::ShiftOrigin(const Length2& newOrigin)
{
    m_world.ShiftOrigin(newOrigin);
}

void Test::KeyboardHandler(KeyID key, KeyAction action, KeyMods mods)
{
    for (const auto& handledKey: m_handledKeys)
    {
        const auto& keyActionMods = handledKey.first;
        if (keyActionMods.key != key)
        {
            continue;
        }
        if (keyActionMods.action != action)
        {
            continue;
        }
        if (mods & keyActionMods.mods)
        {
            continue;
        }
        const auto handlerID = handledKey.second;
        m_keyHandlers[handlerID].second(KeyActionMods{key, action, mods});
    }
}

void Test::RegisterForKey(KeyID key, KeyAction action, KeyMods mods, KeyHandlerID id)
{
    m_handledKeys.push_back(std::make_pair(KeyActionMods{key, action, mods}, id));
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
