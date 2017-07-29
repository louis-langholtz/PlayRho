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

Being a 2D physics engine, it is tempting to use pixels as your units. Unfortunately this may lead to a poorer simulation and less desirable behavior. An object of length 200 pixels would be seen by PlayRho as the size of a 45 story building.

Think of PlayRho bodies as moving billboards upon which you attach your artwork. The billboard may move in a unit system of meters, but you can convert that to pixel coordinates with a scaling factor. You can then use those pixel coordinates to place your sprites, etc. You can also account for flipped coordinate axes.

PlayRho uses radians for angles. The body rotation is stored in radians and may grow unbounded. Consider normalizing the angle of your bodies if the magnitude of the angle becomes too large.

### Factories and Definitions

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

## Worlds

## Bodies

## Joints

## Fixtures

## Shapes

## Contacts

## Limitations

## References
