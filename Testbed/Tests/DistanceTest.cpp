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

#ifndef PLAYRHO_DISTANCE_TEST_HPP
#define PLAYRHO_DISTANCE_TEST_HPP

#include "../Framework/Test.hpp"

#include <sstream>
#include <utility>

namespace testbed {

class DistanceTest : public Test
{
public:
    static inline const auto registered = RegisterTest("Distance Test", MakeUniqueTest<DistanceTest>);

    static Test::Conf GetTestConf()
    {
        auto conf = Test::Conf{};
        conf.settings.drawSkins = true;
        conf.neededSettings = (0x1u << NeedDrawSkinsField);
        conf.description =
            "Demonstrates the collision detection and response between a triangle (Shape A) and an "
            "edge (Shape B) with extra large vertex radii (\"skins\") to help visualize what "
            "happens. The closest points between the two shapes are referred to as \"witness "
            "points\" and are shown in yellow.";
        return conf;
    }

    DistanceTest() : Test(GetTestConf())
    {
        SetGravity(LinearAcceleration2{});

        const auto def = BodyConf{}
                             .UseType(BodyType::Dynamic)
                             .UseLinearDamping(0.9_Hz)
                             .UseAngularDamping(0.9_Hz);
        m_bodyA = CreateBody(GetWorld(), def);
        m_bodyB = CreateBody(GetWorld(), def);

        SetTransform(GetWorld(), m_bodyA, Vec2(-10.0f, 20.2f) * 1_m, 0_deg);
        SetTransform(GetWorld(), m_bodyB,
                     GetLocation(GetWorld(), m_bodyA) + Vec2(19.017401f, 0.13678508f) * 1_m, 0_deg);

        CreateFixtures();
        constexpr auto invalidFixture = std::make_pair(InvalidBodyID, InvalidShapeID);
        RegisterForKey(GLFW_KEY_A, GLFW_PRESS, 0, "Move selected shape left.", [&](KeyActionMods) {
            auto fixtures = GetSelectedFixtures();
            const auto fixture = (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
            const auto body = std::get<BodyID>(fixture);
            if (body != InvalidBodyID) {
                SetTransform(GetWorld(), body, GetLocation(GetWorld(), body) - Length2(0.1_m, 0_m),
                             GetAngle(GetWorld(), body));
                SetAwake(GetWorld(), body);
            }
        });
        RegisterForKey(GLFW_KEY_D, GLFW_PRESS, 0, "Move selected shape right.", [&](KeyActionMods) {
            auto fixtures = GetSelectedFixtures();
            const auto fixture = (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
            const auto body = std::get<BodyID>(fixture);
            if (body != InvalidBodyID) {
                SetTransform(GetWorld(), body, GetLocation(GetWorld(), body) + Length2(0.1_m, 0_m),
                             GetAngle(GetWorld(), body));
                SetAwake(GetWorld(), body);
            }
        });
        RegisterForKey(GLFW_KEY_W, GLFW_PRESS, 0, "Move selected shape up.", [&](KeyActionMods) {
            auto fixtures = GetSelectedFixtures();
            const auto fixture = (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
            const auto body = std::get<BodyID>(fixture);
            if (body != InvalidBodyID) {
                SetTransform(GetWorld(), body, GetLocation(GetWorld(), body) + Vec2(0, 0.1) * 1_m,
                             GetAngle(GetWorld(), body));
                SetAwake(GetWorld(), body);
            }
        });
        RegisterForKey(GLFW_KEY_S, GLFW_PRESS, 0, "Move selected shape down.", [&](KeyActionMods) {
            auto fixtures = GetSelectedFixtures();
            const auto fixture = (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
            const auto body = std::get<BodyID>(fixture);
            if (body != InvalidBodyID) {
                SetTransform(GetWorld(), body, GetLocation(GetWorld(), body) - Vec2(0, 0.1) * 1_m,
                             GetAngle(GetWorld(), body));
                SetAwake(GetWorld(), body);
            }
        });
        RegisterForKey(
            GLFW_KEY_Q, GLFW_PRESS, 0, "Move selected counter-clockwise.", [&](KeyActionMods) {
                auto fixtures = GetSelectedFixtures();
                const auto fixture = (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
                const auto body = std::get<BodyID>(fixture);
                if (body != InvalidBodyID) {
                    SetTransform(GetWorld(), body, GetLocation(GetWorld(), body),
                                 GetAngle(GetWorld(), body) + 5_deg);
                    SetAwake(GetWorld(), body);
                }
            });
        RegisterForKey(GLFW_KEY_E, GLFW_PRESS, 0, "Move selected clockwise.", [&](KeyActionMods) {
            auto fixtures = GetSelectedFixtures();
            const auto fixture = (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
            const auto body = std::get<BodyID>(fixture);
            if (body != InvalidBodyID) {
                SetTransform(GetWorld(), body, GetLocation(GetWorld(), body),
                             GetAngle(GetWorld(), body) - 5_deg);
                SetAwake(GetWorld(), body);
            }
        });
        RegisterForKey(GLFW_KEY_KP_ADD, GLFW_PRESS, 0, "Increase vertex radius of selected shape.",
                       [&](KeyActionMods) {
                           const auto fixtures = GetSelectedFixtures();
                           const auto fixture =
                               (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
                           if (fixture != invalidFixture) {
                               auto conf =
                                   TypeCast<PolygonShapeConf>(GetShape(GetWorld(), fixture.second));
                               conf.UseVertexRadius(conf.vertexRadius + RadiusIncrement);
                               SetShape(GetWorld(), fixture.second, Shape(conf));
                           }
                       });
        RegisterForKey(GLFW_KEY_KP_SUBTRACT, GLFW_PRESS, 0,
                       "Decrease vertex radius of selected shape.", [&](KeyActionMods) {
                           const auto fixtures = GetSelectedFixtures();
                           const auto fixture =
                               (size(fixtures) == 1) ? *(begin(fixtures)) : invalidFixture;
                           if (fixture != invalidFixture) {
                               const auto shape = GetShape(GetWorld(), std::get<ShapeID>(fixture));
                               const auto lastLegitVertexRadius = GetVertexRadius(shape, 0);
                               const auto newVertexRadius = lastLegitVertexRadius - RadiusIncrement;
                               if (newVertexRadius >= 0_m) {
                                   auto conf = TypeCast<PolygonShapeConf>(shape);
                                   conf.UseVertexRadius(newVertexRadius);
                                   SetShape(GetWorld(), fixture.second, Shape(conf));
                               }
                           }
                       });
        RegisterForKey(GLFW_KEY_EQUAL, GLFW_PRESS, 0, "Toggle drawing simplex info.",
                       [&](KeyActionMods) { m_drawSimplexInfo = !m_drawSimplexInfo; });
        RegisterForKey(GLFW_KEY_MINUS, GLFW_PRESS, 0, "Toggle drawing manifold info.",
                       [&](KeyActionMods) { m_drawManifoldInfo = !m_drawManifoldInfo; });
    }

    void CreateFixtures()
    {
        const auto radius = RadiusIncrement * Real{20};
        auto conf = PolygonShapeConf{};
        conf.density = 1_kgpm2;
        conf.vertexRadius = radius;
        auto polygonA = conf;
        // polygonA.SetAsBox(8.0f, 6.0f);
        polygonA.Set({Vec2{-8, -6} * 1_m, Vec2{8, -6} * 1_m, Vec2{0, 6} * 1_m});
        Attach(GetWorld(), m_bodyA, Shape(polygonA));
        conf.vertexRadius = radius * Real{2};
        auto polygonB = conf;
        // polygonB.SetAsBox(7.2_m, 0.8_m);
        polygonB.Set({Vec2{-7.2f, 0} * 1_m, Vec2{+7.2f, 0} * 1_m});
        // polygonB.Set({Vec2{float(-7.2), 0}, Vec2{float(7.2), 0}});
        Attach(GetWorld(), m_bodyB, Shape(polygonB));
    }

    static const auto GetShapeId(const World& world, BodyID body)
    {
        const auto& fixtures = GetShapes(world, body);
        return !empty(fixtures) ? *begin(fixtures) : InvalidShapeID;
    }

    void ShowManifold(Drawer&, const Manifold& manifold, const char* name)
    {
        std::ostringstream strbuf;
        const auto count = manifold.GetPointCount();
        for (auto i = decltype(count){0}; i < count; ++i) {
            strbuf << ", ";
            strbuf << "mp={";
            const auto p = manifold.GetPoint(i);
            strbuf << "lp={" << GetX(p.localPoint) << "," << GetY(p.localPoint) << "}";
            strbuf << ", ";
            strbuf << "cf=" << p.contactFeature;
            strbuf << "}";
        }
        std::ostringstream stream;
        switch (manifold.GetType()) {
        case Manifold::e_circles:
            stream << GetName(manifold.GetType()) << " " << name << ": ";
            stream << "lp={";
            stream << static_cast<double>(Real{GetX(manifold.GetLocalPoint()) / 1_m});
            stream << ",";
            stream << static_cast<double>(Real{GetY(manifold.GetLocalPoint()) / 1_m});
            stream << "}, #=" << unsigned{count} << strbuf.str();
            break;
        case Manifold::e_faceA:
        case Manifold::e_faceB:
            stream << GetName(manifold.GetType()) << " " << name << ": ";
            stream << "lp={";
            stream << static_cast<double>(Real{GetX(manifold.GetLocalPoint()) / 1_m});
            stream << ",";
            stream << static_cast<double>(Real{GetY(manifold.GetLocalPoint()) / 1_m});
            stream << "}, ln={";
            stream << static_cast<double>(GetX(manifold.GetLocalNormal()));
            stream << ",";
            stream << static_cast<double>(GetY(manifold.GetLocalNormal()));
            stream << "}, #=" << unsigned{count} << strbuf.str();
            break;
        default:
            break;
        }
        SetStatus(GetStatus() + stream.str());
    }

    void PostStep(const Settings&, Drawer& drawer) override
    {
        ClearStatus();

        const auto shapeA = GetShape(GetWorld(), GetShapeId(GetWorld(), m_bodyA));
        const auto shapeB = GetShape(GetWorld(), GetShapeId(GetWorld(), m_bodyB));
        const auto proxyA = GetChild(shapeA, 0);
        const auto proxyB = GetChild(shapeB, 0);
        const auto xfmA = GetTransformation(GetWorld(), m_bodyA);
        const auto xfmB = GetTransformation(GetWorld(), m_bodyB);
        const auto maxIndicesAB = GetMaxSeparation(proxyA, xfmA, proxyB, xfmB);
        const auto maxIndicesBA = GetMaxSeparation(proxyB, xfmB, proxyA, xfmA);

        const auto manifold = CollideShapes(proxyA, xfmA, proxyB, xfmB);
#ifdef DEFINE_GET_MANIFOLD
        const auto panifold = GetManifold(proxyA, xfmA, proxyB, xfmB);
#endif

        auto distanceConf = DistanceConf{};
        const auto output = Distance(proxyA, xfmA, proxyB, xfmB, distanceConf);
        distanceConf.cache = Simplex::GetCache(output.simplex.GetEdges());
        const auto witnessPoints = GetWitnessPoints(output.simplex);
        const auto outputDistance =
            GetMagnitude(std::get<0>(witnessPoints) - std::get<1>(witnessPoints));

        const auto rA = proxyA.GetVertexRadius();
        const auto rB = proxyB.GetVertexRadius();
        const auto totalRadius = rA + rB;

        auto adjustedWitnessPoints = witnessPoints;
        auto adjustedDistance = outputDistance;
        if ((outputDistance > totalRadius) && !AlmostZero(outputDistance)) {
            // Shapes are still not overlapped.
            // Move the witness points to the outer surface.
            adjustedDistance -= totalRadius;
            const auto normal =
                GetUnitVector(std::get<1>(witnessPoints) - std::get<0>(witnessPoints));
            std::get<0>(adjustedWitnessPoints) += rA * normal;
            std::get<1>(adjustedWitnessPoints) -= rB * normal;
        }
        else {
            // Shapes are overlapped when radii are considered.
            // Move the witness points to the middle.
            const auto p = (std::get<0>(witnessPoints) + std::get<1>(witnessPoints)) / Real{2};
            std::get<0>(adjustedWitnessPoints) = p;
            std::get<1>(adjustedWitnessPoints) = p;
            adjustedDistance = 0;
        }

        // Draw strings on top of the centers of mass of the shapes (not there origins)...
        drawer.DrawString(GetWorldCenter(GetWorld(), m_bodyA), Drawer::Center, "Shape A");
        drawer.DrawString(GetWorldCenter(GetWorld(), m_bodyB), Drawer::Center, "Shape B");

        std::stringstream os;
        os << "Vertex radii of shapes A & B are ";
        os << static_cast<double>(Real{rA / 1_m}) << "m";
        os << " & ";
        os << static_cast<double>(Real{rB / 1_m}) << "m";
        os << ".\n\n";

        os << "Max separation...\n";
        os << "  For A-face[" << unsigned{GetFirstShapeVertexIdx(maxIndicesAB)} << "]";
        os << " B-vertex[" << unsigned{GetSecondShapeVertexIdx<0>(maxIndicesAB)} << "]: ";
        os << static_cast<double>(Real{maxIndicesAB.distance / 1_m}) << "m.\n";
        os << "  For B-face[" << unsigned{GetFirstShapeVertexIdx(maxIndicesBA)} << "]";
        os << " A-vertex[" << unsigned{GetSecondShapeVertexIdx<0>(maxIndicesBA)} << "]: ";
        os << static_cast<double>(Real{maxIndicesBA.distance / 1_m}) << "m.\n";
        os << "\n";

        if (AlmostEqual(static_cast<double>(Real{maxIndicesAB.distance / 1_m}),
                        static_cast<double>(Real{maxIndicesBA.distance / 1_m}))) {
#ifndef NDEBUG
            const auto childA = GetChild(shapeA, 0);
            const auto childB = GetChild(shapeB, 0);
            // assert(maxIndicesAB.index1 == maxIndicesBA.index2);
            // assert(maxIndicesAB.index2 == maxIndicesBA.index1);
            const auto ifaceA = GetFirstShapeVertexIdx(maxIndicesAB);
            const auto nA = InverseRotate(Rotate(childA.GetNormal(ifaceA), xfmA.q), xfmB.q);
            // shapeA face maxIndicesAB.index1 is coplanar to an edge intersecting shapeB vertex
            // maxIndicesAB.index2
            const auto i1 = GetSecondShapeVertexIdx<0>(maxIndicesAB);
            const auto i0 = GetModuloPrev(i1, childB.GetVertexCount());
            const auto n0 = childB.GetNormal(i0);
            const auto n1 = childB.GetNormal(i1);
            const auto s0 = Dot(nA, n0);
            const auto s1 = Dot(nA, n1);
            assert(s0 != s1);
#if 0
            const auto ifaceB = (s0 < s1)?
                // shapeA face maxIndicesAB.index1 is coplanar to shapeB face i0, and
                // nearest shapeB vertex maxIndicesAB.index2
                i0:
                // shapeA face maxIndicesAB.index1 is coplanar to shapeB face i1, and
                // nearest shapeB vertex maxIndicesAB.index2
                i1;
#endif
#endif
        }
        else if (maxIndicesAB.distance > maxIndicesBA.distance) {
            // shape A face maxIndicesAB.index1 is least separated from shape B vertex
            // maxIndicesAB.index2 Circles or Face-A manifold type.
        }
        else // maxIndicesAB.separation < maxIndicesBA.separation
        {
            // shape B face maxIndicesBA.index1 is least separated from shape A vertex
            // maxIndicesBA.index2 Circles or Face-B manifold type.
        }

        os << "Distance between witness points: ";
        os << static_cast<double>(Real{outputDistance / 1_m}) << "m.\n";
        os << "Min. distance between shapes' skins: ";
        os << static_cast<double>(Real{adjustedDistance / 1_m}) << "m.\n";
        os << "Calculated in " << unsigned{output.iterations};
        os << " iterations on \"" << ToName(output.state) << "\"";
        os << " (max of " << unsigned(distanceConf.maxIterations) << ").\n\n";

        os << "Simplex drawing " << (m_drawSimplexInfo ? "on" : "off") << ".\n";
        {
            const auto numEdges = size(output.simplex);
            os << "Simplex info: size=" << unsigned{numEdges} << ", wpt-a={";
            os << static_cast<double>(Real{GetX(std::get<0>(witnessPoints)) / 1_m});
            os << ",";
            os << static_cast<double>(Real{GetY(std::get<0>(witnessPoints)) / 1_m});
            os << "}, wpt-b={";
            os << static_cast<double>(Real{GetX(std::get<1>(witnessPoints)) / 1_m});
            os << ",";
            os << static_cast<double>(Real{GetY(std::get<1>(witnessPoints)) / 1_m});
            os << "}:\n";
            for (auto i = decltype(numEdges){0}; i < numEdges; ++i) {
                const auto& edge = output.simplex.GetSimplexEdge(i);
                const auto coef = output.simplex.GetCoefficient(i);

                os << "  a[" << unsigned{edge.GetIndexA()} << "]={";
                os << static_cast<double>(Real{GetX(edge.GetPointA()) / 1_m});
                os << ",";
                os << static_cast<double>(Real{GetY(edge.GetPointA()) / 1_m});
                os << "} b[" << unsigned{edge.GetIndexB()} << "]={";
                os << static_cast<double>(Real{GetX(edge.GetPointB()) / 1_m});
                os << ",";
                os << static_cast<double>(Real{GetY(edge.GetPointB()) / 1_m});
                os << "} coef=" << coef << ".\n";
            }
            os << "\n";
        }
        os << "Manifold drawing " << (m_drawManifoldInfo ? "on" : "off") << ".\n";
        SetStatus(GetStatus() + os.str());

        ShowManifold(drawer, manifold, "manifold");
#ifdef DEFINE_GET_MANIFOLD
        ShowManifold(drawer, panifold, "wanifold");
#endif

        if (m_drawManifoldInfo) {
            switch (manifold.GetType()) {
            case Manifold::e_unset:
                break;
            case Manifold::e_circles: {
                const auto pA = Transform(manifold.GetLocalPoint(), xfmA);
                const auto pB = Transform(manifold.GetPoint(0).localPoint, xfmB);
                drawer.DrawCircle(pA, rA / Real{2}, Color(1, 1, 1));
                drawer.DrawCircle(pB, rB / Real{2}, Color(1, 1, 1));
                const auto psm = GetPSM(manifold, 0, xfmA, xfmB);
                const auto psm_separation = psm.m_separation - totalRadius;
                drawer.DrawCircle(psm.m_point, psm_separation, psmPointColor);
                drawer.DrawSegment(psm.m_point, psm.m_point + psm.m_normal * psm_separation,
                                   psmPointColor);
                break;
            }
            case Manifold::e_faceA: {
                const auto pA = Transform(manifold.GetLocalPoint(), xfmA);
                drawer.DrawCircle(pA, rA / Real{2}, Color(1, 1, 1));
                const auto pointCount = manifold.GetPointCount();
                for (auto i = decltype(pointCount){0}; i < pointCount; ++i) {
                    const auto pB = Transform(manifold.GetOpposingPoint(i), xfmB);
                    drawer.DrawCircle(pB, rB / Real{2}, Color(1, 1, 1));
                    const auto psm = GetPSM(manifold, i, xfmA, xfmB);
                    const auto psm_separation = psm.m_separation - totalRadius;
                    drawer.DrawCircle(psm.m_point, psm_separation, psmPointColor);
                    drawer.DrawSegment(psm.m_point, psm.m_point + psm.m_normal * psm_separation,
                                       psmPointColor);
                }
                break;
            }
            case Manifold::e_faceB: {
                const auto pB = Transform(manifold.GetLocalPoint(), xfmB);
                drawer.DrawCircle(pB, rB / Real{2}, Color(1, 1, 1));
                const auto pointCount = manifold.GetPointCount();
                for (auto i = decltype(pointCount){0}; i < pointCount; ++i) {
                    const auto pA = Transform(manifold.GetOpposingPoint(i), xfmA);
                    drawer.DrawCircle(pA, rA / Real{2}, Color(1, 1, 1));
                    const auto psm = GetPSM(manifold, i, xfmA, xfmB);
                    const auto psm_separation = psm.m_separation - totalRadius;
                    drawer.DrawCircle(psm.m_point, psm_separation, psmPointColor);
                    drawer.DrawSegment(psm.m_point, psm.m_point + psm.m_normal * psm_separation,
                                       psmPointColor);
                }
                break;
            }
            }
        }

        if (m_drawSimplexInfo) {
            const auto numEdges = size(output.simplex);
            for (auto i = decltype(numEdges){0}; i < numEdges; ++i) {
                const auto& edge = output.simplex.GetSimplexEdge(i);
                drawer.DrawSegment(edge.GetPointA(), edge.GetPointB(), simplexSegmentColor);
            }

            if (std::get<0>(adjustedWitnessPoints) != std::get<1>(adjustedWitnessPoints)) {
                drawer.DrawPoint(std::get<0>(adjustedWitnessPoints), 4.0f, adjustedPointColor);
                drawer.DrawPoint(std::get<1>(adjustedWitnessPoints), 4.0f, adjustedPointColor);
            }
            else {
                drawer.DrawPoint(std::get<0>(adjustedWitnessPoints), 4.0f, matchingPointColor);
            }

            drawer.DrawPoint(std::get<0>(witnessPoints), 6.0f, witnessPointColor);
            drawer.DrawPoint(std::get<1>(witnessPoints), 6.0f, witnessPointColor);

            for (auto&& edge : output.simplex.GetEdges()) {
                drawer.DrawString(edge.GetPointA(), Drawer::AboveCenter, "Vertex %d",
                                  edge.GetIndexA());
                drawer.DrawString(edge.GetPointB(), Drawer::AboveCenter, "Vertex %d",
                                  edge.GetIndexB());
                drawer.DrawPoint(edge.GetPointA(), 8.0f, simplexPointColor);
                drawer.DrawPoint(edge.GetPointB(), 8.0f, simplexPointColor);
            }
        }
    }

private:
    const Length RadiusIncrement = 2_dm;
    const Color simplexSegmentColor = Color{0.0f, 0.5f, 0.5f}; // dark cyan
    const Color simplexPointColor = Color{0, 1, 1}; // non-transparent cyan
    const Color witnessPointColor = Color{1, 1, 0}; // non-transparent yellow
    const Color adjustedPointColor = Color{1, 0.5f, 0}; // non-transparent light brown
    const Color matchingPointColor = Color{1, 0, 0}; // red
    const Color psmPointColor = Color{0.5f, 1, 1};

    BodyID m_bodyA;
    BodyID m_bodyB;
    bool m_drawSimplexInfo = true;
    bool m_drawManifoldInfo = true;
};

} // namespace testbed

#endif
