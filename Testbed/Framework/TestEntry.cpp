/*
 * Original work Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2021 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include "TestEntry.hpp"
#include "Test.hpp"

namespace testbed {

/// @brief Internal test entries array
///
/// @details Add the registered names of subclasses of Test to this array in the order they should
/// appear in the Testbed.
///
static const char* testEntries[] = {
    "Tiles",
    "Heavy On Light",
    "Heavy On Light Two",
    "Vertical Stack",
    "Bag of Disks",
    "Basic Slider Crank",
    "Slider Crank",
    "Sphere Stack",
    "Convex Hull",
    "Tumbler",
    "Ray-Cast",
    "Dump Shell",
    "Apply Force",
    "Continuous Test",
    "Time of Impact",
    "Motor Joint",
    "Motor Joint Two",
    "One-Sided Platform",
    "Mobile",
    "Mobile Balanced",
    "Conveyor Belt",
    "Gears",
    "Varying Restitution",
    "Cantilever",
    "Character Collision",
    "Fifteen Puzzle",
    "Joints Overview",
    "Edge Test",
    "Body Types",
    "Shape Editing",
    "Car",
    "Prismatic",
    "Revolute",
    "Pulleys",
    "Polygon Shapes",
    "Web",
    "RopeJoint",
    "Pinball",
    "Bullet Test",
    "Confined",
    "Pyramid",
    "Theo Jansen's Walker",
    "Edge Shapes",
    "PolyCollision",
    "Bridge",
    "Breakable",
    "Breakable Two",
    "Chain",
    "Collision Filtering",
    "Collision Processing",
    "Compound Shapes",
    "Distance Test",
    "Dominos",
    "Sensor Test",
    "Spinning Circles",
    "Half Pipe",
    "Encircled Orbiter",
    "Varying Friction",
    "Add Pair Stress Test",
    "Newton's Cradle",
    "Top-down Car",
    "Trajectories",
    "Solar System",
};

std::vector<TestEntry> GetTestEntries()
{
    auto result = std::vector<TestEntry>{};
    const auto creatorMap = GetRegisteredTestMap();
    for (auto&& e: testEntries) {
        if (const auto found = creatorMap.find(e); found != end(creatorMap)) {
            result.push_back(TestEntry{e, found->second});
        }
    }
    return result;
}

} // namespace testbed
