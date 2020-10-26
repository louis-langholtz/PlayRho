/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 * Copyright (c) 2020 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/Dynamics/World.hpp>
#include <PlayRho/Collision/Shapes/PolygonShapeConf.hpp>
#include <PlayRho/Collision/Shapes/DiskShapeConf.hpp>

#include <iostream>
#include <iomanip>

// This is a simpler example of building and running a simulation using PlayRho.
// The code creates a large box to be the ground and a small disk to be a ball
// above the ground. There are no graphics for this example. PlayRho is meant
// to be used with the rendering engine of your game engine.
int main()
{
    // Bring used namespaces into global namespace to simplify this source code.
    using namespace playrho;
    using namespace playrho::d2;

    // Construct a world object which will hold and simulate bodies.
    auto world = World{};

    // Call world's body creation method which allocates memory for ground body and
    // adds it to the world.
    const auto ground = world.CreateBody(BodyConf{}.UseLocation(Length2{0_m, -10_m}));

    // Define the ground shape. Use a polygon configured as a box for this.
    // The extents are the half-width and half-height of the box.
    const auto box = Shape{PolygonShapeConf{}.SetAsBox(50_m, 10_m)};

    // Add the box shape to the ground body.
    world.CreateFixture(FixtureConf{}.UseBody(ground).UseShape(box));

    // Define location above ground for a "dynamic" body & call world's body creation method.
    const auto ball = world.CreateBody(BodyConf{}
                                       .UseLocation(Length2{0_m, 4_m})
                                       .UseType(BodyType::Dynamic)
                                       .UseLinearAcceleration(EarthlyGravity));

    // Define a disk shape for the ball body and create a fixture to add it.
    world.CreateFixture(FixtureConf{}.UseBody(ball).UseShape(DiskShapeConf{}.UseRadius(1_m)));

    // Setup the C++ stream output format.
    std::cout << std::fixed << std::setprecision(2);

    // A little game-like loop.
    for (auto i = 0; i < 60; ++i)
    {
        // Perform a step of the simulation. Keep the time step & iterations fixed.
        // Typically code uses a time step of 1/60 of a second (60Hz). The defaults
        // are setup for that and to generally provide a high enough quality simulation
        // in most scenarios.
        world.Step();

        const auto location = world.GetTransformation(ball).p;
        const auto angle = world.GetAngle(ball);

        // Now print the location and angle of the body.
        std::cout << std::setw(5) << GetX(location);
        std::cout << std::setw(5) << GetY(location);
        std::cout << std::setw(5) << angle;
        std::cout << '\n';
    }

    return 0;
}
