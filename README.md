## What's New

|iOS|Linux|Mac OS X|Windows|
|---|-----|--------|-------|
|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=iosfoo)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=dev)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=macosxfoo)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=win32foo)](https://travis-ci.org/louis-langholtz/Box2D)|

Coverity Scan Build Status: <a href="https://scan.coverity.com/projects/louis-langholtz-box2d"><img alt="Coverity Scan Build Status" src="https://scan.coverity.com/projects/12541/badge.svg"/></a>

This is a fork by Louis Langholtz of the Box2D physics engine originally written by Erin Catto.

General influences on this fork are:
- [C++14](https://en.wikipedia.org/wiki/C%2B%2B14);
- [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md);
- [Unit testing](https://en.wikipedia.org/wiki/Unit_testing);

**NOTE**: These sources make heavy use of the standard C++ library [`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to confirm code correctness.
As such, this library *must* be compiled with the `NDEBUG` preprocessor macro enabled
to see any kind of performance.

Here's a run-down of some of the changes this fork introduces:
- Exported symbols are now within the library namespace of `box2d` and are no longer prefaced by `b2`.
- Mutable global variables in the library have been removed or replaced with runtime-time parameters.
- Preprocessor defines, except those used for include guards, have been replaced with C++ solutions or removed from the API.
- [Rounded and modified corner collisions](Box2D/Documentation/CollisionHandlng.md).
- *Capsule* shapes (using `EdgeShape` or 2-vertex `PolygonShape` instances).
- Support for up to 254-vertex polygons.
- More stable polygon stacking.
- Shared shape ownership, with friction, density, and restitution moved into them (from Fixture class) for reduced memory usage.
- Support for C++11's range-based loops and constant expressions.
- Unit tested via [Google Test](https://github.com/google/googletest/tree/aa148eb2b7f70ede0eb10de34b6254826bfb34f4) and [over 400 tests](https://github.com/louis-langholtz/Box2D/tree/dev/Box2D/UnitTests).
- Continuous integration (CI) building and unit testing of repository updates
  for the Linux platform.
- Compile-time support for zero-runtime strongly-typed physical units (using an interface to [`constexpr`-enhanced boost units](https://github.com/louis-langholtz/units)). For details on how to enable this, see [Documentation/PhysicalUnits.md](Box2D/Documentation/PhysicalUnits.md).
- Compile-time support for `double` and `long double` floating-point types, and 32-bit and 64-bit fixed-point types (in addition to `float`).
- Fully per-step run-time configurable (via [`StepConf`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/StepConf.hpp)).
- In-depth per-step return value statistics (via [`StepStats`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/StepStats.hpp)).
- Increased construction-time configurability of world instances (via [`World::Def`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/World.hpp#L107)).
- Various methods have been rewritten to be [non-member non-friend functions](http://www.drdobbs.com/cpp/how-non-member-functions-improve-encapsu/184401197).
- Various functions and procedures have been rewritten to be ["pure functions"](https://en.wikipedia.org/wiki/Pure_function).
- Testbed enhancements: per-step configurability, per-step statistics, ability to manipulate bodies while paused, and more.
- Testbed test additions: Half Pipe, iforce2d's Topdown Car, Orbiter, Newton's Cradle, and Spinning Circles.

## About

Box2D is a 2D physics engine [licensed](https://github.com/louis-langholtz/Box2D/blob/dev/LICENSE.txt) under a [Zlib License](https://opensource.org/licenses/Zlib).

For help with this fork of Box2D, see https://github.com/louis-langholtz/Box2D/issues.
For help with the original Box2D, please visit http://www.box2d.org.

See [Building.txt](Box2D/Building.txt) to learn how to build Box2D and run the testbed.

To run the demos, set "Testbed" as your startup project and press F5. Some test bed commands are:
- 'r' to reset the current test
- SPACE to launch a bomb
- arrow keys to pan
- 'x' and 'z' to zoom in/out
- use the mouse to click and drag objects
