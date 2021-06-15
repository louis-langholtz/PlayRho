/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "Test.hpp"

#include <PlayRho/Collision/DynamicTree.hpp>

#include "DebugDraw.hpp"

#include <stdio.h>
#include <vector>
#include <sstream>
#include <chrono>
#include <utility>
#include <stdarg.h> // for va_list

#include "imgui.h"
#include "ExtensionsForImgui.hpp"

using namespace playrho;
using namespace playrho::d2;

namespace testbed {

namespace {

constexpr auto RandLimit = 32767;

void DrawCorner(Drawer& drawer, Length2 p, Length r, Angle a0, Angle a1, Color color)
{
    const auto angleDiff = GetRevRotationalAngle(a0, a1);
    auto lastAngle = 0_deg;
    for (auto angle = 5_deg; angle < angleDiff; angle += 5_deg) {
        const auto c0 = p + r * UnitVec::Get(a0 + lastAngle);
        const auto c1 = p + r * UnitVec::Get(a0 + angle);
        drawer.DrawSegment(c0, c1, color);
        lastAngle = angle;
    }
    {
        const auto c0 = p + r * UnitVec::Get(a0 + lastAngle);
        const auto c1 = p + r * UnitVec::Get(a1);
        drawer.DrawSegment(c0, c1, color);
    }
}

void Draw(Drawer& drawer, const DistanceProxy& shape, Color color, bool skins, Transformation xf)
{
    const auto vertexCount = shape.GetVertexCount();
    auto vertices = std::vector<Length2>(vertexCount);
    for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i) {
        vertices[i] = Transform(shape.GetVertex(i), xf);
    }
    const auto fillColor = Color{0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};
    drawer.DrawSolidPolygon(&vertices[0], vertexCount, fillColor);
    drawer.DrawPolygon(&vertices[0], vertexCount, color);

    if (!skins) {
        return;
    }

    const auto skinColor = Color{color.r * 0.6f, color.g * 0.6f, color.b * 0.6f};
    const auto r = GetVertexRadius(shape);
    for (auto i = decltype(vertexCount){0}; i < vertexCount; ++i) {
        if (i > 0) {
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
    if (vertexCount > 1) {
        const auto worldNormal0 = Rotate(shape.GetNormal(vertexCount - 1), xf.q);
        drawer.DrawSegment(vertices[vertexCount - 1] + worldNormal0 * r, vertices[0] + worldNormal0 * r, skinColor);
        const auto worldNormal1 = Rotate(shape.GetNormal(0), xf.q);
        const auto angle0 = GetAngle(worldNormal0);
        const auto angle1 = GetAngle(worldNormal1);
        DrawCorner(drawer, vertices[0], r, angle0, angle1, skinColor);
    }
    else if (vertexCount == 1) {
        DrawCorner(drawer, vertices[0], r, 0_deg, 360_deg, skinColor);
    }
}

void Draw(Drawer& drawer, const Shape& shape, const Color& color, bool skins,
          const Transformation& xf)
{
    const auto type = GetType(shape);
    if (type == GetTypeID<ChainShapeConf>()) {
        Draw(drawer, TypeCast<ChainShapeConf>(shape), color, skins, xf);
        return;
    }
    if (type == GetTypeID<DiskShapeConf>()) {
        Draw(drawer, TypeCast<DiskShapeConf>(shape), color, xf);
        return;
    }
    if (type == GetTypeID<EdgeShapeConf>()) {
        Draw(drawer, TypeCast<EdgeShapeConf>(shape), color, skins, xf);
        return;
    }
    const auto childCount = GetChildCount(shape);
    for (auto i = static_cast<decltype(GetChildCount(shape))>(0); i < childCount; ++i) {
        Draw(drawer, GetChild(shape, i), color, skins, xf);
    }
}

Color GetColor(const World& world, BodyID body)
{
    if (!IsEnabled(world, body))
    {
        return Color{0.5f, 0.5f, 0.3f};
    }
    if (GetType(world, body) == BodyType::Static)
    {
        return Color{0.5f, 0.9f, 0.5f};
    }
    if (GetType(world, body) == BodyType::Kinematic)
    {
        return Color{0.5f, 0.5f, 0.9f};
    }
    if (!IsAwake(world, body))
    {
        return Color{0.75f, 0.75f, 0.75f};
    }
    return Color{0.9f, 0.7f, 0.7f};
}

bool Draw(Drawer& drawer, const World& world, BodyID body,
          bool skins, const Test::FixtureSet& selected)
{
    auto foundBody = false;
    auto foundShape = false;
    const auto bodyColor = GetColor(world, body);
    const auto selectedColor = Brighten(bodyColor, 1.3f);
    const auto xf = GetTransformation(world, body);
    if (Test::Contains(selected, std::make_pair(body, InvalidShapeID))) {
        foundBody = true;
    }
    for (const auto& shapeID: GetShapes(world, body)) {
        auto color = bodyColor;
        if (foundBody || Test::Contains(selected, std::make_pair(body, shapeID))) {
            color = selectedColor;
            foundShape = true;
        }
        Draw(drawer, GetShape(world, shapeID), color, skins, xf);
    }
    return foundBody || foundShape;
}

void Draw(Drawer& drawer, const World& world, JointID id)
{
    const Color color{0.5f, 0.8f, 0.8f};
    const auto& joint = GetJoint(world, id);
    const auto type = GetType(joint);
    if (type == GetTypeID<DistanceJointConf>()) {
        const auto p1 = GetAnchorA(world, id);
        const auto p2 = GetAnchorB(world, id);
        drawer.DrawSegment(p1, p2, color);
    }
    else if (type == GetTypeID<PulleyJointConf>()) {
        const auto p1 = GetAnchorA(world, id);
        const auto p2 = GetAnchorB(world, id);
        const auto s1 = GetGroundAnchorA(world, id);
        const auto s2 = GetGroundAnchorB(world, id);
        drawer.DrawSegment(s1, p1, color);
        drawer.DrawSegment(s2, p2, color);
        drawer.DrawSegment(s1, s2, color);
    }
    else if (type == GetTypeID<TargetJointConf>()) {
        // don't draw this
    }
    else {
        const auto p1 = GetAnchorA(world, id);
        const auto p2 = GetAnchorB(world, id);
        drawer.DrawSegment(p1, p2, color);
        if (const auto bodyId = GetBodyA(joint); bodyId != InvalidBodyID) {
            drawer.DrawSegment(GetTransformation(world, bodyId).p, p1, color);
        }
        if (const auto bodyId = GetBodyB(joint); bodyId != InvalidBodyID) {
            drawer.DrawSegment(GetTransformation(world, bodyId).p, p2, color);
        }
    }
}

void Draw(Drawer& drawer, const AABB& aabb, const Color& color)
{
    Length2 vs[4];
    vs[0] = Length2{aabb.ranges[0].GetMin(), aabb.ranges[1].GetMin()};
    vs[1] = Length2{aabb.ranges[0].GetMax(), aabb.ranges[1].GetMin()};
    vs[2] = Length2{aabb.ranges[0].GetMax(), aabb.ranges[1].GetMax()};
    vs[3] = Length2{aabb.ranges[0].GetMin(), aabb.ranges[1].GetMax()};
    drawer.DrawPolygon(vs, 4, color);
}

inline bool ShouldDrawLabels(const Test::NeededSettings& needed,
                             const Settings& test, const Settings& step)
{
    return (needed & (1u << Test::NeedDrawLabelsField))? test.drawLabels: step.drawLabels;
}

inline bool ShouldDrawSkins(const Test::NeededSettings& needed,
                            const Settings& test, const Settings& step)
{
    return (needed & (1u << Test::NeedDrawSkinsField))? test.drawSkins: step.drawSkins;
}

std::set<BodyID> GetBodySetFromFixtures(const Test::FixtureSet& fixtures)
{
    auto collection = std::set<BodyID>();
    for (const auto& f: fixtures) {
        collection.insert(std::get<BodyID>(f));
    }
    return collection;
}

template <typename T>
struct DequeValuesGetter
{
    static float Func(void* data, int idx)
    {
        const std::deque<T>& deque = *static_cast<std::deque<T>*>(data);
        const auto numElements = size(deque);
        return (idx >= 0 && static_cast<decltype(numElements)>(idx) < numElements)?
        static_cast<float>(deque[static_cast<decltype(numElements)>(idx)]): 0.0f;
    }
};

void ShowStats(const StepConf& stepConf, UiState& ui, const World& world, const Stats& stats)
{
    const auto bodyCount = GetBodyCount(world);
    const auto awakeCount = GetAwakeCount(world);
    const auto sleepCount = bodyCount - awakeCount;
    const auto jointCount = GetJointCount(world);
    const auto attachmentCount = GetAssociationCount(world);
    const auto shapeCount = GetAssociationCount(world);
    const auto touchingCount = GetTouchingCount(world);

    const ImGuiStyle& style = ImGui::GetStyle();
    const auto totalWidth = ImGui::GetWindowWidth() - style.FramePadding.x * 2;
    const auto firstColumnWidth = 65.0f;

    ImGui::Text("Step #=%d (@%fs):", stats.m_stepCount, stats.m_sumDeltaTime);
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("# of steps performed so far for the current test "
                          "and the elapsed simulated time.");
    }
    ImGui::Separator();

    {
        ImGui::ColumnsContext cc(4, "TimesColumns", false);
        ImGui::SetColumnWidth(0, firstColumnWidth);
        ImGui::TextUnformatted("Times:");
        ImGui::NextColumn();
        ImGui::Value("Current", stats.m_curStepDuration.count(), "%f");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Compute time of last step.");
        }
        ImGui::NextColumn();
        ImGui::Value("Max", stats.m_maxStepDuration.count(), "%f");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Maximum compute time of all steps so far for the current test.");
        }
        ImGui::NextColumn();
        ImGui::Value("Sum", stats.m_sumStepDuration.count(), "%f");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Sum compute time of all steps so far for the current test.");
        }
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
            ImGui::SetTooltip("Counts of awake bodies over total bodies.");
        }
        ImGui::NextColumn();
        ImGui::Text("Attachments: %u/%u", shapeCount, attachmentCount);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Counts of shapes over fixtures.");
        }
        ImGui::NextColumn();
        ImGui::Value("Joints", jointCount);
        ImGui::NextColumn();
        ImGui::Text("Contacts: %u/%u(%u/%u)", touchingCount, stats.m_numContacts,
                    stats.m_maxTouching, stats.m_maxContacts);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Counts of contacts touching over total"
                              " (and max touching over max total)."
                              " Click to toggle histogram.");
        }
        if (ImGui::IsItemClicked())
        {
            ui.showContactsHistory = !ui.showContactsHistory;
        }
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(6, "PreStepColumns", false);
        ImGui::SetColumnWidth(0, firstColumnWidth);
        ImGui::TextUnformatted("Pre-step:");
        ImGui::NextColumn();
        ImGui::Value("cts-add", stats.m_stepStats.pre.added);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts added.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted([=]() {
            std::ostringstream os;
            os << "c-ign: " << stats.m_stepStats.pre.ignored << "/"
            << stats.m_sumContactsIgnoredPre;
            return os.str();
        }());
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts ignored over running total ignored.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted([=]() {
            std::ostringstream os;
            os << "c-skip: " << stats.m_stepStats.pre.skipped << "/"
            << stats.m_sumContactsSkippedPre;
            return os.str();
        }());
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts skipped over running total skipped.");
        }
        ImGui::NextColumn();
        ImGui::Value("c-del", stats.m_stepStats.pre.destroyed);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts deleted.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted([=]() {
            std::ostringstream os;
            os << "c-upd: " << stats.m_stepStats.pre.updated << "/" <<
            stats.m_sumContactsUpdatedPre;
            return os.str();
        }());
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts updated over running total updated.");
        }
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
            ImGui::SetTooltip("Contacts added.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("i-find");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Islands found.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("i-solv");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Islands solved.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("posit");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Position iterations.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("velit");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Velocity iterations.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("pmov");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Proxies moved.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("minsep");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Minimum separation.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("maxP");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Max incremental impulse.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("b-slept");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Bodies slept.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("c-find");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts found.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("c-@maxs");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts at max substeps.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("c-upd");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Contacts whose TOI was updated.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("max-d-it");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Max distance iterations.");
        }
        ImGui::NextColumn();
        ImGui::TextUnformatted("max-t-it");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Max TOI iterations.");
        }
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("Reg. step:");
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.reg.contactsAdded);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.reg.islandsFound);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.reg.islandsSolved);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.reg.sumPosIters);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.reg.sumVelIters);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.proxiesMoved);
        ImGui::NextColumn();
        ImGui::Text("%f", static_cast<double>(Real{stats.m_stepStats.reg.minSeparation / Meter}));
        ImGui::NextColumn();
        ImGui::Text("%.2f", static_cast<double>(Real{
            stats.m_stepStats.reg.maxIncImpulse / NewtonSecond}));
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.reg.bodiesSlept);
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("TOI step:");
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.contactsAdded);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.islandsFound);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.islandsSolved);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.sumPosIters);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.sumVelIters);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.proxiesMoved);
        ImGui::NextColumn();
        ImGui::Text("%f", static_cast<double>(Real{
            stats.m_stepStats.toi.minSeparation / Meter}));
        ImGui::NextColumn();
        ImGui::Text("%.2f", static_cast<double>(Real{
            stats.m_stepStats.toi.maxIncImpulse / NewtonSecond}));
        ImGui::NextColumn();
        // Skip bodies slept column
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.contactsFound);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.contactsAtMaxSubSteps);
        ImGui::NextColumn();
        ImGui::Text("%u", stats.m_stepStats.toi.contactsUpdatedToi);
        ImGui::NextColumn();
        ImGui::Text("%u", unsigned{stats.m_stepStats.toi.maxDistIters});
        ImGui::NextColumn();
        ImGui::Text("%u", unsigned{stats.m_stepStats.toi.maxToiIters});
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("Reg. sums:");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Regular step sums.");
        }
        ImGui::NextColumn();
        // Skip c-add column
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumRegIslandsFound));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumRegIslandsSolved));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumRegPosIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumRegVelIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumRegProxiesMoved));
        ImGui::NextColumn();
    }

    {
        ImGui::ColumnsContext cc(15, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("TOI sums:");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("TOI step sums.");
        }
        ImGui::NextColumn();
        // Skip c-add column
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumToiIslandsFound));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumToiIslandsSolved));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumToiPosIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumToiVelIters));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumToiProxiesMoved));
        ImGui::NextColumn();
        // Skip minSeparation column
        ImGui::NextColumn();
        // Skip maxIncImpulse column
        ImGui::NextColumn();
        // Skip bodies slept column
        ImGui::NextColumn();
        // Skip contacts found column
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumContactsAtMaxSubSteps));
        ImGui::NextColumn();
        ImGui::TextUnformatted(std::to_string(stats.m_sumContactsUpdatedToi));
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
        stream << "min-sep=" << static_cast<double>(Real{stats.m_minRegSep / Meter});
        stream << ", max-sep=" << static_cast<double>(Real{stats.m_maxRegSep / Meter});
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
        stream << "min-sep=" << static_cast<double>(Real{stats.m_minToiSep / Meter});
        stream << ", max-dist-iter=" << unsigned{stats.m_maxDistIters} << "/" << unsigned{stepConf.maxDistanceIters};
        stream << ", max-toi-iter=" << unsigned{stats.m_maxToiIters} << "/" << unsigned{stepConf.maxToiIters};
        stream << ", max-root-iter=" << unsigned{stats.m_maxRootIters} << "/" << unsigned{stepConf.maxToiRootIters};
        stream << ", max-simul-cts=" << stats.m_maxSimulContacts;
        stream << ".";
        ImGui::TextUnformatted(stream.str());
        ImGui::NextColumn();
    }

    {
        const auto leafCount = GetTree(world).GetLeafCount();
        const auto nodeCount = GetTree(world).GetNodeCount();
        const auto height = GetHeight(GetTree(world));
        const auto imbalance = GetMaxImbalance(GetTree(world));
        const auto quality = ComputePerimeterRatio(GetTree(world));
        const auto capacity = GetTree(world).GetNodeCapacity();

        ImGui::ColumnsContext cc(2, nullptr, false);
        ImGui::SetColumnWidths(totalWidth, {firstColumnWidth});
        ImGui::TextUnformatted("Dyn. tree:");
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Broad-phase dynamic tree statistics.");
        }
        ImGui::NextColumn();
        ImGui::Text("nodes=%u/%u/%u, ", leafCount, nodeCount, capacity);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Leaf/node/capacity counts.");
        }
        ImGui::SameLine(0, 0);
        ImGui::Text("height=%u, ", height);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Height of the tree (lower is better).");
        }
        ImGui::SameLine(0, 0);
        ImGui::Text("imbal=%u, ", imbalance);
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Maximum imbalance of branch nodes (lower is better).");
        }
        ImGui::SameLine(0, 0);
        ImGui::Text("p-rat=%.2f, ", static_cast<double>(quality));
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Perimeter ratio (lower is better).");
        }
        ImGui::SameLine(0, 0);
        std::ostringstream stream;
        stream << stats.m_maxAABB;
        ImGui::Text("max-aabb=%s.", stream.str().c_str());
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("Maximum Axis Aligned Bounding Box (AABB).");
        }
        ImGui::NextColumn();
    }
}

void DrawContactInfo(Drawer& drawer, const Settings& settings,
                     const Test::FixtureSet& selectedFixtures,
                     const Test::ContactPoints& points)
{
    const auto k_impulseScale = 0.1_s / 1_kg;
    const auto k_axisScale = 0.3_m;
    const auto addStateColor = Color{0.3f, 0.9f, 0.3f}; // greenish
    const auto persistStateColor = Color{0.3f, 0.3f, 0.9f}; // blueish
    const auto contactNormalColor = Color{0.7f, 0.7f, 0.7f}; // light gray
    const auto normalImpulseColor = Color{0.9f, 0.9f, 0.3f}; // yellowish
    const auto frictionImpulseColor = Color{0.9f, 0.9f, 0.3f}; // yellowish
    const auto lighten = 1.3f;
    const auto darken = 0.9f;
    for (const auto& point: points)
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
            const auto p2_left = p2 - headLength * Rotate(point.normal, UnitVec::GetTopRight());
            const auto p2_right = p2 - headLength * Rotate(point.normal, UnitVec::GetBottomRight());
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

bool DrawWorld(Drawer& drawer, const World& world, const Test::FixtureSet& selected,
               const Test::NeededSettings& needed,
               const Settings& testSettings, const Settings& stepSettings)
{
    auto found = false;

    if (stepSettings.drawShapes) {
        const auto drawLabels = ShouldDrawLabels(needed, testSettings, stepSettings);
        const auto drawSkins = ShouldDrawSkins(needed, testSettings, stepSettings);
        for (const auto& b: GetBodies(world)) {
            if (Draw(drawer, world, b, drawSkins, selected)) {
                found = true;
            }
            if (drawLabels) {
                // Use center of mass instead of body center since body center may not
                drawer.DrawString(GetWorldCenter(world, b), Drawer::Center, "%d",
                                  GetWorldIndex(world, b));
            }
        }
    }

    if (stepSettings.drawJoints) {
        for (const auto& j: GetJoints(world)) {
            Draw(drawer, world, j);
        }
    }

    if (stepSettings.drawAABBs) {
        const auto color = Color{0.9f, 0.3f, 0.9f};
        const auto root = GetTree(world).GetRootIndex();
        if (root != DynamicTree::GetInvalidSize()) {
            const auto worldAabb = GetTree(world).GetAABB(root);
            Draw(drawer, worldAabb, color);
            Query(GetTree(world), worldAabb, [&](DynamicTree::Size id) {
                Draw(drawer, GetTree(world).GetAABB(id), color);
                return DynamicTreeOpcode::Continue;
            });
        }
    }

    if (stepSettings.drawCOMs) {
        const auto k_axisScale = 0.4_m;
        const auto red = Color{1.0f, 0.0f, 0.0f};
        const auto green = Color{0.0f, 1.0f, 0.0f};
        for (const auto& b: GetBodies(world)) {
            const auto massScale = std::pow(static_cast<float>(StripUnit(GetMass(world, b))), 1.0f/3);
            auto xf = GetTransformation(world, b);
            xf.p = GetWorldCenter(world, b);
            const auto p1 = xf.p;
            drawer.DrawSegment(p1, p1 + massScale * k_axisScale * GetXAxis(xf.q), red);
            drawer.DrawSegment(p1, p1 + massScale * k_axisScale * GetYAxis(xf.q), green);
        }
    }

    return found;
}

} // namespace

const std::map<TypeID, const char*> Test::shapeTypeToNameMap = {
    std::make_pair(GetTypeID<ChainShapeConf>(), "Chain"),
    std::make_pair(GetTypeID<DiskShapeConf>(), "Disk"),
    std::make_pair(GetTypeID<EdgeShapeConf>(), "Edge"),
    std::make_pair(GetTypeID<MultiShapeConf>(), "MultiShape"),
    std::make_pair(GetTypeID<PolygonShapeConf>(), "Polygon"),
};

const std::map<TypeID, const char*> Test::jointTypeToNameMap = {
    std::make_pair(GetTypeID<RevoluteJointConf>(), "Revolute"),
    std::make_pair(GetTypeID<PrismaticJointConf>(), "Prismatic"),
    std::make_pair(GetTypeID<DistanceJointConf>(), "Distance"),
    std::make_pair(GetTypeID<PulleyJointConf>(), "Pulley"),
    std::make_pair(GetTypeID<TargetJointConf>(), "Target"),
    std::make_pair(GetTypeID<GearJointConf>(), "Gear"),
    std::make_pair(GetTypeID<WheelJointConf>(), "Wheel"),
    std::make_pair(GetTypeID<WeldJointConf>(), "Weld"),
    std::make_pair(GetTypeID<FrictionJointConf>(), "Friction"),
    std::make_pair(GetTypeID<RopeJointConf>(), "Rope"),
    std::make_pair(GetTypeID<MotorJointConf>(), "Motor"),
};

const char* Test::ToName(TypeID type) noexcept
{
    if (const auto found = jointTypeToNameMap.find(type); found != end(jointTypeToNameMap)) {
        return found->second;
    }
    if (const auto found = shapeTypeToNameMap.find(type); found != end(shapeTypeToNameMap)) {
        return found->second;
    }
    const auto name = GetName(type);
    if (std::strstr(name, "playrho::d2::Rectangle")) {
        return "Rectangle";
    }
    return name;
}

bool Test::AlertUser(const std::string& title, const char* fmt, ...)
{
    ImGui::OpenPopup(title.c_str());
    if (const auto opened = ImGui::PopupModalContext(title.c_str())) {
        va_list args;
        va_start(args, fmt);
        ImGui::TextWrappedV(fmt, args);
        va_end(args);
        if (ImGui::Button("Close")) {
            ImGui::CloseCurrentPopup();
            return true;
        }
    }
    return false;
}

const char* Test::ToName(DistanceOutput::State value)
{
    switch (value) {
    case DistanceOutput::MaxPoints:
        return "MaxPoints";
    case DistanceOutput::UnfitSearchDir:
        return "UnfitSearchDir";
    case DistanceOutput::DuplicateIndexPair:
        return "DuplicateIndexPair";
    case DistanceOutput::HitMaxIters:
        return "HitMaxIters";
    case DistanceOutput::Unknown:
        break;
    }
    return "Unknown";
}

const LinearAcceleration2 Test::Gravity = LinearAcceleration2{
    Real(0.0f) * MeterPerSquareSecond,
    -Real(10.0f) * MeterPerSquareSecond
};

bool Test::Contains(const FixtureSet& fixtures, const std::pair<BodyID, ShapeID>& f) noexcept
{
    return fixtures.find(f) != std::end(fixtures);
}

void Test::DestructionListenerImpl::SayGoodbye(JointID joint) noexcept
{
    if (test->m_targetJoint == joint)
    {
        test->m_targetJoint = InvalidJointID;
    }
    else
    {
        test->JointDestroyed(joint);
    }
}

Test::Test(Conf conf):
    m_world(conf.worldConf),
    m_neededSettings(conf.neededSettings),
    m_settings(conf.settings),
    m_description(conf.description),
    m_credits(conf.credits),
    m_seeAlso(conf.seeAlso),
    m_numContactsPerStep(m_maxHistory, 0u),
    m_numTouchingPerStep(m_maxHistory, 0u)
{
    m_destructionListener.test = this;
    SetShapeDestructionListener(m_world, [this](ShapeID id){
        m_destructionListener.SayGoodbye(id);
    });
    SetJointDestructionListener(m_world, [this](JointID id){
        m_destructionListener.SayGoodbye(id);
    });
    SetPreSolveContactListener(m_world, [this](ContactID id, const Manifold& manifold) {
        PreSolve(id, manifold);
    });
}

Test::~Test()
{
}

void Test::ResetWorld(const World &saved)
{
    ClearSelectedFixtures();

    auto bombIndex = static_cast<decltype(size(GetBodies(m_world)))>(-1);

    {
        auto i = decltype(size(GetBodies(m_world))){0};
        for (const auto& b: GetBodies(m_world))
        {
            if (b == m_bomb)
            {
                bombIndex = i;
            }
            ++i;
        }
    }

    m_world = saved;

    {
        auto i = decltype(size(GetBodies(m_world))){0};
        for (const auto& b: GetBodies(m_world))
        {
            if (i == bombIndex)
            {
                m_bomb = b;
            }
            ++i;
        }
    }
}

void Test::PreSolve(ContactID contactId, const Manifold& oldManifold)
{
    const auto pointStates = GetPointStates(oldManifold, GetManifold(m_world, contactId));
    const auto worldManifold = GetWorldManifold(m_world, contactId);

    ContactPoint cp;
    const auto& contact = GetContact(m_world, contactId);
    cp.bodyIdA = GetBodyA(contact);
    cp.shapeIdA = GetShapeA(contact);
    cp.bodyIdB = GetBodyB(contact);
    cp.shapeIdB = GetShapeB(contact);
    cp.normal = worldManifold.GetNormal();

    const auto count = worldManifold.GetPointCount();
    for (auto i = decltype(count){0}; i < count; ++i)
    {
        const auto ci = worldManifold.GetImpulses(i);
        cp.normalImpulse = get<0>(ci);
        cp.tangentImpulse = get<1>(ci);
        cp.state = pointStates.state2[i];
        cp.position = worldManifold.GetPoint(i);
        cp.separation = worldManifold.GetSeparation(i);
        m_points.push_back(cp);
    }
}

void Test::SetSelectedFixtures(const FixtureSet& value) noexcept
{
    m_selectedFixtures = value;
    m_selectedBodies = GetBodySetFromFixtures(value);
}

void Test::MouseDown(const Length2& p)
{
    m_mouseWorld = p;

    if (m_targetJoint != InvalidJointID) {
        return;
    }

    // Make a small box.
    const auto aabb = GetFattenedAABB(AABB{p}, 1_m / 1000);

    // Query the world for overlapping shapes.
    auto fixtures = FixtureSet{};
    Query(aabb, [this,&p,&fixtures](BodyID b, ShapeID f, const ChildCounter) {
        if (TestPoint(m_world, b, f, p)) {
            fixtures.insert(std::make_pair(b, f));
        }
        return true; // Continue the query.
    });

    SetSelectedFixtures(fixtures);
    if (size(fixtures) == 1) {
        const auto body = std::get<BodyID>(*begin(fixtures));
        if (GetType(m_world, body) == BodyType::Dynamic) {
            auto md = TargetJointConf{};
            md.bodyB = body;
            md.target = p;
            md.localAnchorB = GetLocalPoint(GetBody(m_world, body), p);
            md.maxForce = Real(10000) * GetMass(m_world, body) * MeterPerSquareSecond;
            m_targetJoint = CreateJoint(m_world, md);
            SetAwake(m_world, body);
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

    const auto deltaTime = m_lastDeltaTime;
    const auto relP = m_bombSpawnPoint - p;
    const auto vel = (deltaTime != 0_s)? LinearVelocity2{
        Real{0.25f} * GetX(relP) / deltaTime,
        Real{0.25f} * GetY(relP) / deltaTime
    }: LinearVelocity2{};
    LaunchBomb(m_bombSpawnPoint, vel);
    m_bombSpawning = false;
}

void Test::ShiftMouseDown(const Length2& p)
{
    m_mouseWorld = p;

    if (m_targetJoint != InvalidJointID)
    {
        return;
    }

    SpawnBomb(p);
}

void Test::MouseUp(const Length2& p)
{
    if (m_targetJoint != InvalidJointID)
    {
        Destroy(m_world, m_targetJoint);
        m_targetJoint = InvalidJointID;
    }

    if (m_bombSpawning)
    {
        CompleteBombSpawn(p);
    }
}

void Test::MouseMove(const Length2& p)
{
    m_mouseWorld = p;

    if (m_targetJoint != InvalidJointID)
    {
        SetTarget(m_world, m_targetJoint, p);
    }
}

void Test::LaunchBomb()
{
    const auto deltaTime = m_lastDeltaTime;
    const auto viewport = ConvertScreenToWorld();
    const auto worldX = RandomFloat(viewport.ranges[0].GetMin()/1_m, viewport.ranges[0].GetMax()/1_m);
    const auto atA = Length2{worldX * 1_m, viewport.ranges[1].GetMax()};
    const auto centerX = GetCenter(viewport.ranges[0]);
    const auto height = GetSize(viewport.ranges[1]);
    const auto atB = Length2{centerX, viewport.ranges[1].GetMax() - (height * 9.0f / 10.0f)};
    const auto v = (deltaTime != 0_s)? (atB - atA) / (deltaTime * 30): LinearVelocity2{};
    LaunchBomb(atA, v);
}

void Test::LaunchBomb(const Length2& at, const LinearVelocity2 v)
{
    if (m_bomb != InvalidBodyID) {
        const auto shapes = GetShapes(m_world, m_bomb); // copy shape identifiers
        Destroy(m_world, m_bomb);
        for (auto& shape: shapes) {
            Destroy(m_world, shape);
        }
    }

    m_bomb = CreateBody(m_world, BodyConf{}.UseType(BodyType::Dynamic).UseBullet(true)
                                .UseLocation(at).UseLinearVelocity(v)
                                .UseLinearAcceleration(m_gravity));

    auto conf = DiskShapeConf{};
    conf.vertexRadius = m_bombRadius;
    conf.density = m_bombDensity;
    conf.restitution = 0.0f;
    Attach(m_world, m_bomb, CreateShape(m_world, conf));
}

void Test::Step(const Settings& settings, Drawer& drawer, UiState& ui)
{
    m_lastDeltaTime = settings.dt * Second;
    m_textLine = 3 * DRAW_STRING_NEW_LINE;

    PreStep(settings, drawer);

    if (settings.pause)
    {
        if ((settings.dt == 0) && m_targetJoint != InvalidJointID)
        {
            const auto bodyB = GetBodyB(m_world, m_targetJoint);
            const auto anchorB = GetAnchorB(m_world, m_targetJoint);
            const auto centerB = GetLocation(m_world, bodyB);
            const auto destB = GetTarget(m_world, m_targetJoint);
            //m_points.clear();
            SetTransform(m_world, bodyB, destB - (anchorB - centerB), GetAngle(m_world, bodyB));
        }
    }

    if (settings.dt != 0)
    {
        // Resets point count for contact point accumalation.
        m_points.clear();
    }

    SetSubStepping(m_world, settings.enableSubStepping);

    auto stepConf = StepConf{};

    stepConf.deltaTime = settings.dt * Second;

    stepConf.regVelocityIters = static_cast<StepConf::iteration_type>(settings.regVelocityIterations);
    stepConf.regPositionIters = static_cast<StepConf::iteration_type>(settings.regPositionIterations);
    stepConf.toiVelocityIters = static_cast<StepConf::iteration_type>(settings.toiVelocityIterations);
    stepConf.toiPositionIters = static_cast<StepConf::iteration_type>(settings.toiPositionIterations);

    stepConf.maxSubSteps = static_cast<StepConf::iteration_type>(settings.maxSubSteps);
    stepConf.maxToiRootIters = static_cast<StepConf::iteration_type>(settings.maxToiRootIters);

    stepConf.minStillTimeToSleep = static_cast<Real>(settings.minStillTimeToSleep) * Second;
    stepConf.maxTranslation = static_cast<Real>(settings.maxTranslation) * Meter;
    stepConf.maxRotation = Real{settings.maxRotation} * Degree;

    stepConf.linearSlop = Real{settings.linearSlop} * Meter;
    stepConf.angularSlop = Real{settings.angularSlop} * Degree;
    stepConf.regMinSeparation = Real{settings.regMinSeparation} * Meter;
    stepConf.toiMinSeparation = Real{settings.toiMinSeparation} * Meter;
    stepConf.targetDepth = 3 * settings.linearSlop * Meter;
    stepConf.tolerance = settings.tolerance * Meter;
    stepConf.aabbExtension = settings.aabbExtension * Meter;

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
    const auto stepStats = ::playrho::d2::Step(m_world, stepConf);
    const auto end = std::chrono::system_clock::now();

    m_stats.m_maxAABB = GetEnclosingAABB(m_stats.m_maxAABB, GetAABB(GetTree(m_world)));
    
    m_stats.m_sumContactsUpdatedPre += stepStats.pre.updated;
    m_stats.m_sumContactsIgnoredPre += stepStats.pre.ignored;
    m_stats.m_sumContactsSkippedPre += stepStats.pre.skipped;

    m_stats.m_sumRegIslandsFound += stepStats.reg.islandsFound;
    m_stats.m_sumRegIslandsSolved += stepStats.reg.islandsSolved;
    m_stats.m_sumRegPosIters += stepStats.reg.sumPosIters;
    m_stats.m_sumRegVelIters += stepStats.reg.sumVelIters;
    m_stats.m_sumRegProxiesMoved += stepStats.reg.proxiesMoved;

    m_stats.m_sumToiIslandsFound += stepStats.toi.islandsFound;
    m_stats.m_sumToiIslandsSolved += stepStats.toi.islandsSolved;
    m_stats.m_sumToiPosIters += stepStats.toi.sumPosIters;
    m_stats.m_sumToiVelIters += stepStats.toi.sumVelIters;
    m_stats.m_sumToiProxiesMoved += stepStats.toi.proxiesMoved;
    m_stats.m_sumContactsUpdatedToi += stepStats.toi.contactsUpdatedToi;
    m_stats.m_sumToiContactsUpdatedTouching += stepStats.toi.contactsUpdatedTouching;
    m_stats.m_sumToiContactsSkippedTouching += stepStats.toi.contactsSkippedTouching;
    m_stats.m_sumContactsAtMaxSubSteps += stepStats.toi.contactsAtMaxSubSteps;

    m_stats.m_maxSimulContacts = std::max(m_stats.m_maxSimulContacts,
                                          stepStats.toi.maxSimulContacts);
    m_stats.m_maxDistIters = std::max(m_stats.m_maxDistIters, stepStats.toi.maxDistIters);
    m_stats.m_maxRootIters = std::max(m_stats.m_maxRootIters, stepStats.toi.maxRootIters);
    m_stats.m_maxToiIters = std::max(m_stats.m_maxToiIters, stepStats.toi.maxToiIters);

    if (stepStats.reg.minSeparation < std::numeric_limits<Length>::infinity())
    {
        m_stats.m_minRegSep = std::min(m_stats.m_minRegSep, stepStats.reg.minSeparation);
        m_stats.m_maxRegSep = std::max(m_stats.m_maxRegSep, stepStats.reg.minSeparation);
    }

    if (settings.dt != 0)
    {
        m_stats.m_sumDeltaTime += settings.dt;

        ++m_stats.m_stepCount;
        m_stats.m_stepStats = stepStats;
        m_stats.m_minToiSep = std::min(m_stats.m_minToiSep, stepStats.toi.minSeparation);

        m_stats.m_curStepDuration = end - start;
        m_stats.m_maxStepDuration = std::max(m_stats.m_maxStepDuration, m_stats.m_curStepDuration);
        m_stats.m_sumStepDuration += m_stats.m_curStepDuration;
    }

    m_stats.m_numContacts = GetContactCount(m_world);
    m_stats.m_maxContacts = std::max(m_stats.m_maxContacts, m_stats.m_numContacts);
    
    if (size(m_numContactsPerStep) >= m_maxHistory)
    {
        m_numContactsPerStep.pop_front();
    }
    m_numContactsPerStep.push_back(m_stats.m_numContacts);

    if (ui.showStats)
    {
        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_Appearing);
        ImGui::SetNextWindowSize(ImVec2(600, 300), ImGuiCond_Appearing);
        ImGui::WindowContext wc("Step Statistics", &ui.showStats, ImGuiWindowFlags_NoCollapse);
        if (size(m_numTouchingPerStep) >= m_maxHistory)
        {
            m_numTouchingPerStep.pop_front();
        }
        const auto touchingCount = GetTouchingCount(m_world);
        m_numTouchingPerStep.push_back(touchingCount);
        m_stats.m_maxTouching = std::max(m_stats.m_maxTouching, touchingCount);
        ShowStats(stepConf, ui, m_world, m_stats);
    }
    
    if (ui.showContactsHistory)
    {
        ImGui::SetNextWindowPos(ImVec2(10, 200), ImGuiCond_Once);
        ImGui::WindowContext wc("Contacts Histogram", &ui.showContactsHistory,
                                ImGuiWindowFlags_NoCollapse);
        char buffer[40];
        
        std::sprintf(buffer, "Max of %u", m_stats.m_maxTouching);
        ImGui::PlotHistogram("# Touching", DequeValuesGetter<std::size_t>::Func,
                             &m_numTouchingPerStep, static_cast<int>(size(m_numTouchingPerStep)),
                             0, buffer, 0.0f, static_cast<float>(m_stats.m_maxContacts),
                             ImVec2(600, 100));

        std::sprintf(buffer, "Max of %u", m_stats.m_maxContacts);
        ImGui::PlotHistogram("# Contacts", DequeValuesGetter<std::size_t>::Func,
                             &m_numContactsPerStep, static_cast<int>(size(m_numContactsPerStep)),
                             0, buffer, 0.0f, static_cast<float>(m_stats.m_maxContacts),
                             ImVec2(600, 100));
    }

    if (m_targetJoint != InvalidJointID)
    {
        const auto p1 = GetAnchorB(m_world, m_targetJoint);
        const auto p2 = GetTarget(m_world, m_targetJoint);
        drawer.DrawPoint(p1, 4.0f, Color{0.0f, 1.0f, 0.0f});
        drawer.DrawPoint(p2, 4.0f, Color{0.0f, 1.0f, 0.0f});
        drawer.DrawSegment(p1, p2, Color{0.8f, 0.8f, 0.8f});
    }

    if (m_bombSpawning)
    {
        drawer.DrawPoint(m_bombSpawnPoint, 4.0f, Color{0.0f, 0.0f, 1.0f});
        drawer.DrawSegment(m_mouseWorld, m_bombSpawnPoint, Color{0.8f, 0.8f, 0.8f});
    }

    DrawContactInfo(drawer, settings, GetSelectedFixtures(), GetPoints());

    PostStep(settings, drawer);

    const auto selectedFixtures = GetSelectedFixtures();
    const auto selectedFound = DrawWorld(drawer, m_world, selectedFixtures,
                                         GetNeededSettings(), GetSettings(), settings);
    if (!empty(selectedFixtures) && !selectedFound)
    {
        ClearSelectedFixtures();
    }

    drawer.Flush();
}

void Test::ShiftOrigin(const Length2& newOrigin)
{
    ::playrho::d2::ShiftOrigin(m_world, newOrigin);
}

void Test::KeyboardHandler(KeyID key, KeyAction action, KeyMods mods)
{
    for (const auto& handledKey: m_handledKeys)
    {
        const auto& keyActionMods = std::get<0>(handledKey);
        if (keyActionMods.key != key)
        {
            continue;
        }
        if (keyActionMods.action != action)
        {
            continue;
        }
        if (mods != keyActionMods.mods)
        {
            continue;
        }
        const auto handlerID = std::get<1>(handledKey);
        std::get<1>(m_keyHandlers[handlerID])(KeyActionMods{key, action, mods});
    }
}

void Test::RegisterForKey(KeyID key, KeyAction action, KeyMods mods, KeyHandlerID id)
{
    m_handledKeys.push_back(std::make_pair(KeyActionMods{key, action, mods}, id));
}

void Test::Query(const AABB& aabb, QueryShapeCallback callback)
{
    ::playrho::d2::Query(GetTree(m_world), aabb, callback);
}

// Exported free functions...

Real RandomFloat()
{
    auto r = static_cast<Real>(std::rand() & RandLimit);
    r /= RandLimit;
    r = 2.0f * r - 1.0f;
    return r;
}

Real RandomFloat(Real lo, Real hi)
{
    auto r = static_cast<Real>(std::rand() & RandLimit);
    r /= RandLimit;
    r = (hi - lo) * r + lo;
    return r;
}

void Draw(Drawer& drawer, const DiskShapeConf& shape, Color color, const Transformation& xf)
{
    const auto center = Transform(shape.GetLocation(), xf);
    const auto radius = shape.GetRadius();
    const auto fillColor = Color{0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f};
    drawer.DrawSolidCircle(center, radius, fillColor);
    drawer.DrawCircle(center, radius, color);

    // Draw a line fixed in the circle to animate rotation.
    const auto axis = Rotate(Vec2{1, 0}, xf.q);
    drawer.DrawSegment(center, center + radius * axis, color);
}

void Draw(Drawer& drawer, const EdgeShapeConf& shape, Color color, bool skins,
          const Transformation& xf)
{
    const auto v1 = Transform(shape.GetVertexA(), xf);
    const auto v2 = Transform(shape.GetVertexB(), xf);
    drawer.DrawSegment(v1, v2, color);

    if (skins)
    {
        const auto r = GetVertexRadius(shape);
        if (r > 0_m)
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

void Draw(Drawer& drawer, const ChainShapeConf& shape, Color color, bool skins,
          const Transformation& xf)
{
    const auto count = shape.GetVertexCount();
    const auto r = GetVertexRadius(shape);
    const auto skinColor = Color{color.r * 0.6f, color.g * 0.6f, color.b * 0.6f};

    auto v1 = Transform(shape.GetVertex(0), xf);
    for (auto i = decltype(count){1}; i < count; ++i)
    {
        const auto v2 = Transform(shape.GetVertex(i), xf);
        drawer.DrawSegment(v1, v2, color);
        if (skins && r > 0_m)
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

bool HasFixture(const Test::ContactPoint& cp, const Test::FixtureSet& fixtures) noexcept
{
    for (auto fixture: fixtures) {
        if (fixture == std::make_pair(cp.bodyIdA, cp.shapeIdA) ||
            fixture == std::make_pair(cp.bodyIdB, cp.shapeIdB)) {
            return true;
        }
    }
    return false;
}

} // namespace testbed
