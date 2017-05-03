/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/Box2D
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

#include "../Framework/TestEntry.hpp"

#include "AddPair.hpp"
#include "ApplyForce.hpp"
#include "BasicSliderCrank.hpp"
#include "BodyTypes.hpp"
#include "Breakable.hpp"
#include "Bridge.hpp"
#include "BulletTest.hpp"
#include "Cantilever.hpp"
#include "Car.hpp"
#include "ContinuousTest.hpp"
#include "Chain.hpp"
#include "CharacterCollision.hpp"
#include "CollisionFiltering.hpp"
#include "CollisionProcessing.hpp"
#include "CompoundShapes.hpp"
#include "Confined.hpp"
#include "ConvexHull.hpp"
#include "ConveyorBelt.hpp"
#include "DistanceTest.hpp"
#include "Dominos.hpp"
#include "DumpShell.hpp"
#include "DynamicTreeTest.hpp"
#include "EdgeShapes.hpp"
#include "EdgeTest.hpp"
#include "Gears.hpp"
#include "HeavyOnLight.hpp"
#include "HeavyOnLightTwo.hpp"
#include "Mobile.hpp"
#include "MobileBalanced.hpp"
#include "MotorJoint.hpp"
#include "OneSidedPlatform.hpp"
#include "Pinball.hpp"
#include "PolyCollision.hpp"
#include "PolyShapes.hpp"
#include "Prismatic.hpp"
#include "Pulleys.hpp"
#include "Pyramid.hpp"
#include "RayCast.hpp"
#include "Revolute.hpp"
#include "RopeJoint.hpp"
//#include "Rope.hpp"
#include "SensorTest.hpp"
#include "ShapeEditing.hpp"
#include "SliderCrank.hpp"
#include "SphereStack.hpp"
#include "TheoJansen.hpp"
#include "Tiles.hpp"
#include "TimeOfImpact.hpp"
#include "Tumbler.hpp"
#include "VaryingFriction.hpp"
#include "VaryingRestitution.hpp"
#include "VerticalStack.hpp"
#include "Web.hpp"
#include "SpinningCircle.hpp"
#include "HalfPipe.hpp"
#include "Orbiter.hpp"
#include "NewtonsCradle.hpp"
#include "iforce2d_TopdownCar.hpp"

namespace box2d {

/// @brief Internal test entries array
///
/// @detail Add subclasses of Test to this array in the order they should appear in the
///   Testbed.
///
static const TestEntry testEntries[] =
{
    {"Tiles", MakeUniqueTest<Tiles>},
    {"Heavy on Light", MakeUniqueTest<HeavyOnLight>},
    {"Heavy on Light Two", MakeUniqueTest<HeavyOnLightTwo>},
    {"Vertical Stack", MakeUniqueTest<VerticalStack>},
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
    {"Edge Test", MakeUniqueTest<EdgeTest>},
    {"Body Types", MakeUniqueTest<BodyTypes>},
    {"Shape Editing", MakeUniqueTest<ShapeEditing>},
    {"Car", MakeUniqueTest<Car>},
    {"Prismatic", MakeUniqueTest<Prismatic>},
    {"Revolute", MakeUniqueTest<Revolute>},
    {"Pulleys", MakeUniqueTest<Pulleys>},
    {"Polygon Shapes", MakeUniqueTest<PolyShapes>},
    {"Web", MakeUniqueTest<Web>},
    // {"Rope", MakeUniqueTest<RopeTest>},
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

    /// Empty entry signifying end of array.
    {nullptr, nullptr}
};

const TestEntry* GetTestEntries()
{
    return testEntries;
}

} // namespace box2d
