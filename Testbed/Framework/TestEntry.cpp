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

#include "TestEntry.hpp"

#include "../Tests/AddPair.hpp"
#include "../Tests/ApplyForce.hpp"
#include "../Tests/BagOfDisks.hpp"
#include "../Tests/BasicSliderCrank.hpp"
#include "../Tests/BodyTypes.hpp"
#include "../Tests/Breakable.hpp"
#include "../Tests/BreakableTwo.hpp"
#include "../Tests/Bridge.hpp"
#include "../Tests/BulletTest.hpp"
#include "../Tests/Cantilever.hpp"
#include "../Tests/Car.hpp"
#include "../Tests/ContinuousTest.hpp"
#include "../Tests/Chain.hpp"
#include "../Tests/CharacterCollision.hpp"
#include "../Tests/CollisionFiltering.hpp"
#include "../Tests/CollisionProcessing.hpp"
#include "../Tests/CompoundShapes.hpp"
#include "../Tests/Confined.hpp"
#include "../Tests/ConvexHull.hpp"
#include "../Tests/ConveyorBelt.hpp"
#include "../Tests/DistanceTest.hpp"
#include "../Tests/Dominos.hpp"
#include "../Tests/DumpShell.hpp"
#include "../Tests/DynamicTreeTest.hpp"
#include "../Tests/EdgeShapes.hpp"
#include "../Tests/EdgeTest.hpp"
#include "../Tests/FifteenPuzzle.hpp"
#include "../Tests/Gears.hpp"
#include "../Tests/HeavyOnLight.hpp"
#include "../Tests/HeavyOnLightTwo.hpp"
#include "../Tests/Mobile.hpp"
#include "../Tests/MobileBalanced.hpp"
#include "../Tests/MotorJoint.hpp"
#include "../Tests/OneSidedPlatform.hpp"
#include "../Tests/Pinball.hpp"
#include "../Tests/PolyCollision.hpp"
#include "../Tests/PolyShapes.hpp"
#include "../Tests/Prismatic.hpp"
#include "../Tests/Pulleys.hpp"
#include "../Tests/Pyramid.hpp"
#include "../Tests/RayCast.hpp"
#include "../Tests/Revolute.hpp"
#include "../Tests/RopeJoint.hpp"
#include "../Tests/SensorTest.hpp"
#include "../Tests/ShapeEditing.hpp"
#include "../Tests/SliderCrank.hpp"
#include "../Tests/SphereStack.hpp"
#include "../Tests/TheoJansen.hpp"
#include "../Tests/Tiles.hpp"
#include "../Tests/TimeOfImpact.hpp"
#include "../Tests/Tumbler.hpp"
#include "../Tests/VaryingFriction.hpp"
#include "../Tests/VaryingRestitution.hpp"
#include "../Tests/VerticalStack.hpp"
#include "../Tests/Web.hpp"
#include "../Tests/SpinningCircle.hpp"
#include "../Tests/HalfPipe.hpp"
#include "../Tests/Orbiter.hpp"
#include "../Tests/NewtonsCradle.hpp"
#include "../Tests/iforce2d_TopdownCar.hpp"

namespace playrho {

/// @brief Internal test entries array
///
/// @details Add subclasses of Test to this array in the order they should appear in the
///   Testbed.
///
static const TestEntry testEntries[] =
{
    {"Tiles", MakeUniqueTest<Tiles>},
    {"Heavy on Light", MakeUniqueTest<HeavyOnLight>},
    {"Heavy on Light Two", MakeUniqueTest<HeavyOnLightTwo>},
    {"Vertical Stack", MakeUniqueTest<VerticalStack>},
    {"Bag of Disks", MakeUniqueTest<BagOfDisks>},
    {"Basic Slider Crank", MakeUniqueTest<BasicSliderCrank>},
    {"Slider Crank", MakeUniqueTest<SliderCrank>},
    {"Sphere Stack", MakeUniqueTest<SphereStack>},
    {"Convex Hull", MakeUniqueTest<ConvexHull>},
    {"Tumbler", MakeUniqueTest<Tumbler>},
    {"Ray-Cast", MakeUniqueTest<class RayCast>},
    {"Dump Shell", MakeUniqueTest<DumpShell>},
    {"Apply Force", MakeUniqueTest<class ApplyForce>},
    {"Continuous Test", MakeUniqueTest<ContinuousTest>},
    {"Time of Impact", MakeUniqueTest<TimeOfImpactTest>},
    {"Motor Joint", MakeUniqueTest<MotorJointTest>},
    {"One-Sided Platform", MakeUniqueTest<OneSidedPlatform>},
    {"Mobile", MakeUniqueTest<Mobile>},
    {"MobileBalanced", MakeUniqueTest<MobileBalanced>},
    {"Conveyor Belt", MakeUniqueTest<ConveyorBelt>},
    {"Gears", MakeUniqueTest<Gears>},
    {"Varying Restitution", MakeUniqueTest<VaryingRestitution>},
    {"Cantilever", MakeUniqueTest<Cantilever>},
    {"Character Collision", MakeUniqueTest<CharacterCollision>},
    {"Fifteen Puzzle", MakeUniqueTest<FifteenPuzzle>},
    {"Edge Test", MakeUniqueTest<EdgeTest>},
    {"Body Types", MakeUniqueTest<BodyTypes>},
    {"Shape Editing", MakeUniqueTest<ShapeEditing>},
    {"Car", MakeUniqueTest<Car>},
    {"Prismatic", MakeUniqueTest<Prismatic>},
    {"Revolute", MakeUniqueTest<Revolute>},
    {"Pulleys", MakeUniqueTest<Pulleys>},
    {"Polygon Shapes", MakeUniqueTest<PolyShapes>},
    {"Web", MakeUniqueTest<Web>},
    {"RopeJoint", MakeUniqueTest<RopeJointTest>},
    {"Pinball", MakeUniqueTest<Pinball>},
    {"Bullet Test", MakeUniqueTest<BulletTest>},
    {"Confined", MakeUniqueTest<Confined>},
    {"Pyramid", MakeUniqueTest<Pyramid>},
    {"Theo Jansen's Walker", MakeUniqueTest<TheoJansen>},
    {"Edge Shapes", MakeUniqueTest<EdgeShapes>},
    {"PolyCollision", MakeUniqueTest<PolyCollision>},
    {"Bridge", MakeUniqueTest<Bridge>},
    {"Breakable", MakeUniqueTest<Breakable>},
    {"Breakable Two", MakeUniqueTest<BreakableTwo>},
    {"Chain", MakeUniqueTest<Chain>},
    {"Collision Filtering", MakeUniqueTest<CollisionFiltering>},
    {"Collision Processing", MakeUniqueTest<CollisionProcessing>},
    {"Compound Shapes", MakeUniqueTest<CompoundShapes>},
    {"Distance Test", MakeUniqueTest<DistanceTest>},
    {"Dominos", MakeUniqueTest<Dominos>},
    {"Dynamic Tree", MakeUniqueTest<DynamicTreeTest>},
    {"Sensor Test", MakeUniqueTest<SensorTest>},
    {"Spinning Circles", MakeUniqueTest<SpinningCircle>},
    {"Half Pipe", MakeUniqueTest<HalfPipe>},
    {"Orbiter", MakeUniqueTest<Orbiter>},
    {"Varying Friction", MakeUniqueTest<VaryingFriction>},
    {"Add Pair Stress Test", MakeUniqueTest<AddPair>},
    {"Newton's Cradle", MakeUniqueTest<NewtonsCradle>},
    {"Top-down Car", MakeUniqueTest<iforce2d_TopdownCar>},
};

static const std::size_t numTestEntries = sizeof(testEntries) / sizeof(TestEntry);

Span<const TestEntry> GetTestEntries()
{
    // Intentionally encapsulates access to the static array through this function.
    // This may help avoid issues with startup-time dependencies of having global data defined
    // in a different file than it's used in.
    // This also allows this code to use sizeof to compile-time determine the size of
    // the test entries array and to return it along with the array pointer as a span.
    return Span<const TestEntry>(testEntries, numTestEntries);
}

} // namespace playrho
