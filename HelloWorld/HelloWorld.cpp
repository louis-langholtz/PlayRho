/*
 * Original work Copyright (c) 2007 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2023 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <iostream>
#include <iomanip>

#include <playrho/d2/BasicAPI.hpp>

// This is a simpler example of building and running a simulation using PlayRho.
// The code creates a large box to be the ground and a small disk to be a ball
// above the ground. There are no graphics for this example. PlayRho is meant
// to be used with the rendering engine of your game engine.
int main()
{
    // Brings used namespaces into global namespace to simplify this source code.
    using namespace playrho;
    using namespace playrho::d2;

    // Just some named constant values used herein - to avoid "magic numbers"
    constexpr auto boxX = +0_m;
    constexpr auto boxY = -10_m;
    constexpr auto boxWidth = 50_m;
    constexpr auto bodyHeight = 10_m;
    constexpr auto maxSteps = 60;
    constexpr auto textWidth = 5;
    constexpr auto fixedPrecision = 2;
    constexpr auto ballStart = Length2{0_m, 4_m};

    // Constructs a world object which will hold and simulate bodies.
    auto world = World{};

    // Creates a shape within the world that's to act like the ground.
    // Uses a polygon configured as a box for this.
    // The extents are the half-width and half-height of the box.
    const auto box = CreateShape(world, PolygonShapeConf{}.SetAsBox(boxWidth, bodyHeight));

    // Creates a body within the world that's attached to the ground-like box shape
    // and that's centered 10 meters below the origin.
    CreateBody(world, BodyConf{}.Use(box).UseLocation(Length2{boxX, boxY}));

    // Creats a 1 meter radius, ball-like, disk shape within the world.
    const auto diskShape = CreateShape(world, DiskShapeConf{}.UseRadius(1_m));

    // Creates "dynamic" body having disk shape that fall's to ground within world.
    const auto ball = CreateBody(world, BodyConf{}
                                            .Use(BodyType::Dynamic)
                                            .Use(diskShape)
                                            .UseLocation(ballStart)
                                            .UseLinearAcceleration(EarthlyGravity));

    // Setup the C++ stream output format.
    std::cout << std::fixed << std::setprecision(fixedPrecision);

    // A little game-like loop.
    for (auto i = 0; i < maxSteps; ++i) {
        // Perform a step of the simulation. Keep the time step & iterations fixed.
        // Typically code uses a time step of 1/60 of a second (60Hz). The defaults
        // are setup for that and to generally provide a high enough quality simulation
        // in most scenarios.
        Step(world);

        const auto transformation = GetTransformation(world, ball);
        const auto location = GetLocation(transformation);
        const auto angle = GetAngle(transformation);

        // Now print the location and angle of the body.
        // The Y value will drop from 4.00 to 1.00 as the ball drops to the ground.
        std::cout << std::setw(textWidth) << GetX(location);
        std::cout << std::setw(textWidth) << GetY(location);
        std::cout << std::setw(textWidth) << angle;
        std::cout << '\n';
    }

    return 0;
}
