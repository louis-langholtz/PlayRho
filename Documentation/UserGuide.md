# PlayRho User Guide

This is the user guide for PlayRho. This document is based off of the Box2D 2.3.0 User Manual written by Erin Catto. For information on Box2D, please see: http://box2d.org.

## Introduction

### About

PlayRho is a real-time oriented physics engine and library. It's currently best suited for 2-D games.

Programmers can use it in their games to make objects move in some more realistic ways and make the game world more interactive. From a game engine's point of view, a physics engine is just a system for procedural animation.

PlayRho is written in ISO C++14. Most of the types defined in the engine are within the `playrho` namespace. Hopefully this is sufficient to avoid name clashing with your game engine.

### Prerequisites

This document assumes that you are familiar with basic physics concepts such as mass, force, torque, and impulses. If not, please first consult Google search and Wikipedia.

Since PlayRho is written in C++14, you are expected to be experienced in C++14 programming. PlayRho should not be your first C++ programming project. You should be comfortable with compiling, linking, and debugging.

### About this Document

This document covers some of the PlayRho API. However, not every aspect is covered. You are encouraged to look at the code for the applications included with PlayRho to learn more. Also, the PlayRho code base has comments formatted for Doxygen, so there is also a hyper-linked [API document](https://louis-langholtz.github.io/PlayRho/API/index.html) which is more specific to the API itself.

### Feedback and Reporting Bugs

If you have a question or feedback about PlayRho, please create an issue for it on GitHub. This is also a place for community discussion.

PlayRho issues are tracked using the GitHub [Issues interface for this project](https://github.com/louis-langholtz/PlayRho/issues). This provides a way to track issues and helps to ensure that your issue will not be lost.

Please file bugs and feature requests here: https://github.com/louis-langholtz/PlayRho/issues

You can help with getting your issue fixed by providing more details. A source code contribution in the form of a [Google Test](https://github.com/google/googletest) based C++ unit test that reproduces the problem is highly encouraged.

### Core Concepts

PlayRho works with several fundamental concepts and objects. We briefly define these objects here and more details are given later in this document.

- *shape* A shape is 2D geometrical object, such as a circle or polygon.
- *rigid body* A chunk of matter that is so strong that the distance between any two bits of matter on the chunk is constant. They are hard like a diamond. In the following discussion we use body interchangeably with rigid body.
- *fixture* A fixture binds a shape to a body and adds material properties such as density, friction, and restitution. A fixture puts a shape into the collision system (broad-phase) so that it can collide with other shapes.
- *constraint* A constraint is a physical connection that removes degrees of freedom from bodies. A 2D body has 3 degrees of freedom (two translation coordinates and one rotation coordinate). If we take a body and pin it to the wall (like a pendulum) we have constrained the body to the wall. At this point the body can only rotate about the pin, so the constraint has removed 2 degrees of freedom.
- *contact constraint* A special constraint designed to prevent penetration of rigid bodies and to simulate friction and restitution. You do not create contact constraints; they are created automatically by PlayRho.
- *joint* This is a constraint used to hold two or more bodies together. PlayRho supports several joint types: revolute, prismatic, distance, and more. Some joints may have limits and motors.
- *joint limit* A joint limit restricts the range of motion of a joint. For example, the human elbow only allows a certain range of angles.
- *joint motor* A joint motor drives the motion of the connected bodies according to the joint's degrees of freedom. For example, you can use a motor to drive the rotation of an elbow.
- *world* A physics world is a collection of bodies, fixtures, and constraints that interact together. PlayRho supports the creation of multiple worlds, but this is usually not necessary or desirable.
- *solver* The physics world has a solver that is used to advance time and to resolve contact and joint constraints. The PlayRho solver is meant to be a higher performance iterative solver that operates in order N time, where N is the number of constraints.
- *continuous collision* The solver advances bodies in time using discrete time steps. Without intervention this can lead to tunneling. PlayRho contains specialized algorithms to deal with tunneling. First, the collision algorithms can interpolate the motion of two bodies to find the first time of impact (TOI). Second, there is a sub-stepping solver that moves bodies to their first time of impact and then resolves the collision.

### Modules

PlayRho is composed of three modules: Common, Collision, and Dynamics. The Common module has code for allocation, math, and settings. The Collision module defines shapes, a broad-phase, and collision functions/queries. Finally the Dynamics module provides the simulation world, bodies, fixtures, and joints.

### Units

PlayRho works with floating point numbers and tolerances have to be used to try to make PlayRho perform well enough for things like games. These tolerances have been tuned to work better with meters-kilogram-second (MKS) units. In particular, PlayRho has been tuned to work best with moving shapes between 0.1 and 10 meters. So this means objects between soup cans and buses in size are more ideally suited for the default tuning. Static shapes may be up to 50 meters long without trouble.

Being a 2-dimensional physics engine, it is tempting to use pixels as your units. Unfortunately this may lead to a poorer simulation and less desirable behavior. An object of length 200 pixels would be seen by PlayRho as the size of a 45 story building.

Think of PlayRho bodies as moving billboards upon which you attach your artwork. The billboard may move in a unit system of meters, but you can convert that to pixel coordinates with a scaling factor. You can then use those pixel coordinates to place your sprites, etc. You can also account for flipped coordinate axes.

PlayRho uses radians for angles. The body rotation is stored in radians and may grow unbounded. Consider normalizing the angle of your bodies if the magnitude of the angle becomes too large.

### Factories and Definitions

Memory management plays a central role in the design of the PlayRho API. So when you create a `Body` or `Joint`, you need to call the factory functions on `World`. You should never try to allocate these types in another manner.
There are creation functions:

    Body* World::CreateBody();
    Joint* World::CreateJoint(const JointDef& def);

And there are corresponding destruction functions:

    void World::Destroy(Body* body);
    void World::Destroy(Joint* joint);

When you create a body or joint, you need to provide a definition. These definitions contain all the information needed to build the body or joint. By using this approach we can prevent construction errors, keep the number of function parameters small, provide sensible defaults, and reduce the number of accessors.

Since fixtures (shapes) must be parented to a body, they are created and destroyed using a factory method on `Body`:

    Fixture* Body::CreateFixture(const FixtureDef& def);
    void Body::DestroyFixture(Fixture* fixture);

There is also shortcut to create a fixture directly from the shape and density.

    Fixture* Body::CreateFixture(const Shape& shape, Density density);

Factories do not retain references to the definitions. So you can create definitions on the stack and keep them in temporary resources.

## HelloWorld Example

In the distribution of PlayRho is a Hello World console application. The program creates a large ground box and a small dynamic box. This code does not contain any graphics. All you will see is text output in the console of the box's position over time.

This is a simpler example of how to get up and running with PlayRho.

### Creating a World

Every PlayRho program begins with the creation of a `World` object. `World` is the physics hub that manages memory, objects, and simulation. You can allocate the physics world locally, dynamically, or globally.

A world object can be created simply by default construction like:

    playrho::World world;

Or for [Almost Always Auto](https://herbsutter.com/2013/08/12/gotw-94-solution-aaa-style-almost-always-auto/) fans as:

    auto world = playrho::World{};

Note that in the `HelloWorld` example, we are creating the world on the stack, so the world must remain in scope.

So now that we have our physics world, let's start adding some stuff to it.

### Creating a Ground Body

Bodies are built using the following steps:

1. Define a body with position, damping, etc.
2. Use the world object to create the body.
3. Define fixtures with a shape, friction, density, etc.
4. Create fixtures on the body.

For step 1 we create the ground body. For this we need a body definition. With the body definition we specify the initial position of the ground body.

For step 2 the body definition is passed to the world object to create the ground body. The world object does not keep a reference to the body definition. Bodies are static by default. Static bodies don't collide with other static bodies and are immovable.

### Creating a Dynamic Body

So now we have a ground body. We can use the same technique to create a dynamic body. The main difference, besides dimensions, is that we must establish the dynamic body's mass properties.
First we create the body using `CreateBody`. By default bodies are static, so we should set the `BodyType` at construction time to make the body dynamic.

    Body* body = world.CreateBody(BodyDef{}.UseBodyType(BodyType::Dynamic).UseLocation(Length2D{Real(0) * Meter, Real(4) * Meter}));

Next we create and attach a polygon shape using a fixture definition. First we create a box shape:

    PolygonShape dynamicBox;
    dynamicBox.SetAsBox(Real(1) * Meter, Real(1) * Meter);

Next we create a fixture definition using the box. Notice that we set density to 1. The default density is zero. Also, the friction on the shape is set to 0.3.

    FixtureDef fixtureDef;
    fixtureDef.shape = &amp;dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;

Using the fixture definition we can now create the fixture. This automatically updates the mass of the body. You can add as many fixtures as you like to a body. Each one contributes to the total mass.

    body->CreateFixture(&fixtureDef);

That's it for initialization. We are now ready to begin simulating.

### Simulating the World (of PlayRho)

So we have initialized the ground box and a dynamic box. Now we are ready to set Newton loose to do his thing. We just have a couple more issues to consider.

PlayRho uses a computational algorithm called an integrator. Integrators simulate the physics equations at discrete points of time. This goes along with the traditional game loop where we essentially have a flip book of movement on the screen. So we need to pick a time step for Box2D. Generally physics engines for games like a time step at least as fast as 60Hz or 1/60 seconds. You can get away with larger time steps, but you will have to be more careful about setting up the definitions for your world. We also don't like the time step to change much. A variable time step produces variable results, which makes it difficult to debug. So don't tie the time step to your frame rate (unless you really, really have to). Without further ado, here is the time step.

    float32 timeStep = 1.0f / 60.0f;

In addition to the integrator, PlayRho also uses a larger bit of code called a constraint solver. The constraint solver solves all the constraints in the simulation, one at a time. A single constraint can be solved perfectly. However, when we solve one constraint, we slightly disrupt other constraints. To get a good solution, we need to iterate over all constraints a number of times.

There are two phases in the constraint solver: a velocity phase and a position phase. In the velocity phase the solver computes the impulses necessary for the bodies to move correctly. In the position phase the solver adjusts the positions of the bodies to reduce overlap and joint detachment. Each phase has its own iteration count. In addition, the position phase may exit iterations early if the errors are small

The suggested iteration count for PlayRho is 8 for velocity and 3 for position. You can tune this number to your liking, just keep in mind that this has a trade-off between performance and accuracy. Using fewer iterations increases performance but accuracy suffers. Likewise, using more iterations decreases performance but improves the quality of your simulation. For this simple example, we don't need much iteration. Here are our chosen iteration counts.

    int32 velocityIterations = 6;
    int32 positionIterations = 2;

Note that the time step and the iteration count are completely unrelated. An iteration is not a sub-step. One solver iteration is a single pass over all the constraints within a time step. You can have multiple passes over the constraints within a single time step.

We are now ready to begin the simulation loop. In your game the simulation loop can be merged with your game loop. In each pass through your game loop you call World::Step. Just one call is usually enough, depending on your frame rate and your physics time step.

The Hello World program was designed to be simple, so it has no graphical output. The code prints out the position and rotation of the dynamic body. Here is the simulation loop that simulates 60 time steps for a total of 1 second of simulated time.

    for (int32 i = 0; i < 60; ++i) {
        world.Step(timeStep, velocityIterations, positionIterations);
        Vec2 position = body->GetPosition();
        float32 angle = body->GetAngle();
        printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
    }

The output shows the box falling and landing on the ground box. Your output should look like this:

    0.00 4.00 0.00
    0.00 3.99 0.00
    0.00 3.98 0.00
    ...
    0.00 1.25 0.00
    0.00 1.13 0.00
    0.00 1.01 0.00

### Cleanup

When a world leaves scope or is deleted by calling delete on a pointer, all the memory reserved for bodies, fixtures, and joints is freed. This is done to improve performance and make your life easier. However, you will need to nullify any body, fixture, or joint pointers you have because they will become invalid.

### The Testbed

Once you have conquered the HelloWorld example, you should start looking at PlayRho's Testbed. The
  Testbed is a testing framework and demo environment. Here are some of the features:

- Camera with pan and zoom.
- Mouse picking of shapes attached to dynamic bodies.
- Extensible set of tests.
- GUI for selecting tests, parameter tuning, and debug drawing options.
- Pause and single step simulation.
- Text rendering.

The Testbed has many examples of PlayRho usage in the test cases and the framework itself. I encourage you to explore and tinker with the testbed as you learn PlayRho.

Note: the Testbed is written using freeglut and GLUI. The Testbed is not part of the PlayRho library. The PlayRho library is agnostic about rendering. As shown by the HelloWorld example, you don't need a renderer to use PlayRho.

## Common

### About

The Common module contains settings, memory management, and vector math.

### Settings

The header `Settings.hpp` contains:
- Types such as int32 and `Real`
- Constants
- Allocation wrappers
- The version number

### Types

PlayRho defines various types such as float32, int8, etc. to make it easy to determine the size of structures.

### Constants

PlayRho defines several constants. These are all documented in `Settings.hpp`. Normally you do not need to adjust these constants.

PlayRho typically uses floating point math for collision and simulation. Due to round-off error some numerical tolerances are defined. Some tolerances are absolute and some are relative. Absolute tolerances use MKS units.

### Allocation wrappers

The settings file defines `Alloc` and `Free` for large allocations. You may forward these calls to your own memory management system.

### Version

The `Version` structure holds the current version so you can query this at run-time.

### Math

PlayRho includes a simple small vector and matrix module. This has been designed to suit the internal needs of PlayRho and the API. All the members are exposed, so you may use them freely in your application.
The math library is kept simple to make PlayRho easy to port and maintain.

## Collision Module

### About

The Collision module contains shapes and functions that operate on them. The module also contains a dynamic tree and broad-phase to acceleration collision processing of large systems.

The collision module is designed to be usable outside of the dynamic system. For example, you can use the dynamic tree for other aspects of your game besides physics.

However, the main purpose of PlayRho is to provide a rigid body physics engine, so the using the collision module by itself may feel limited for some applications. Likewise, I will not make a strong effort to document it or polish the APIs.

### Shapes

Shapes describe collision geometry and may be used independently of physics simulation. At a minimum, you should understand how to create shapes that can be later attached to rigid bodies.

PlayRho shapes implement the Shape base class. The base class defines functions to:
- Test a point for overlap with the shape.
- Perform a ray cast against the shape.
- Compute the shape's AABB.
- Compute the mass properties of the shape.

In addition, each shape has a type member and a radius. The radius even applies to polygons, as discussed below.

Keep in mind that a shape does not know about bodies and stand apart from the dynamics system. Shapes are stored in a compact form that is optimized for size and performance. As such, shapes are not easily moved around. You have to manually set the shape vertex positions to move a shape. However, when a shape is attached to a body using a fixture, the shapes move rigidly with the host body. In summary:

- When a shape is not attached to a body, you can view it’s vertices as being expressed in world- space.
- When a shape is attached to a body, you can view it’s vertices as being expressed in local coordinates.

#### Disk Shapes
#### Polygon Shapes
#### Edge Shapes
#### Chain Shapes

### Unary Geometric Queries

You can perform a couple geometric queries on a single shape.

#### Shape Point Test

You can test a point for overlap with a shape. You provide the shape and a local point.

    const auto point = Length2D{0, 0};
    const auto hit = TestPoint(shape, point);

## Worlds

## Bodies

## Joints

## Fixtures

## Shapes

## Contacts

## Limitations

## References
