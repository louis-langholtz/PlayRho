/*
 * Original work Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 * Modified work Copyright (c) 2017 Louis Langholtz https://github.com/louis-langholtz/PlayRho
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

#include <PlayRho/PlayRho.hpp>
#include <cstdio>

// This is a simple example of building and running a simulation using PlayRho.
// Here we create a large ground box and a small dynamic box.
// There are no graphics for this example. PlayRho is meant to be used with your
// rendering engine in your game engine.
int main(int, char**)
{
    // Construct a world object, which will hold and simulate the rigid bodies.
    box2d::World world;

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    const auto groundBody = world.CreateBody(box2d::BodyDef{}
                                             .UseLocation(box2d::Vec2(0.0, -10.0) * box2d::Meter));

    // Define the ground box shape.
    // The extents are the half-widths of the box.
    const auto groundBox = std::make_shared<box2d::PolygonShape>(box2d::Real(50) * box2d::Meter,
                                                                 box2d::Real(10) * box2d::Meter);

    // Add the ground fixture to the ground body.
    groundBody->CreateFixture(groundBox);

    // Define the dynamic body. We set its position and call the body factory.
    const auto body = world.CreateBody(box2d::BodyDef{}
                                       .UseType(box2d::BodyType::Dynamic)
                                       .UseLocation(box2d::Vec2(0.0, 4.0) * box2d::Meter));

    // Define another box shape for our dynamic body.
    const auto dynamicBox = std::make_shared<box2d::PolygonShape>(box2d::Real(1) * box2d::Meter,
                                                                  box2d::Real(1) * box2d::Meter);

    // Set the box density to be non-zero, so it will be dynamic.
    dynamicBox->SetDensity(box2d::Real(1) * box2d::KilogramPerSquareMeter);

    // Override the default friction.
    dynamicBox->SetFriction(box2d::Real(0.3));

    // Add the shape to the body.
    body->CreateFixture(dynamicBox);

    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    auto stepConf = box2d::StepConf{};
    stepConf.SetInvTime(box2d::Real(60) * box2d::Hertz);
    stepConf.regVelocityIterations = 6;
    stepConf.regPositionIterations = 2;

    std::printf("%4.2s %4.2s %4.2s\n", "x", "y", "a");

    // This is our little game loop.
    for (auto i = 0; i < 60; ++i)
    {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world.Step(stepConf);

        // Now print the position and angle of the body.
        const auto position = body->GetLocation();
        const auto angle = body->GetAngle();

        std::printf("%4.2f %4.2f %4.2f\n",
                    double(box2d::Real(position.x / box2d::Meter)),
                    double(box2d::Real(position.y / box2d::Meter)),
                    double(box2d::Real(angle / box2d::Degree)));
    }

    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned pointers, so be careful about your world management.

    return 0;
}
