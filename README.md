##What's New

###Overview

This is a fork by Louis Langholtz of the Box2D physics engine.

General influences on this fork are:
- [C++11](https://en.wikipedia.org/wiki/C%2B%2B11) & [C++14](https://en.wikipedia.org/wiki/C%2B%2B14);
- [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md);
- [Unit testing](https://en.wikipedia.org/wiki/Unit_testing);
 
**NOTE**: These sources make heavy use of the standard C++ library [`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to confirm code correctness.
As such, this library *must* be compiled with the `NDEBUG` preprocessor macro enabled
to see any kind of performance.

###Features

Here's some of the changes that might be considered "new features":
- Exported symbols are all within the library namespace of `box2d` (and no longer preficed by `b2`).
- All preprocessor defines - with the exception of those for include guards - have been removed from the API or replaced with C++ solutions.
- Rounded corner collisions.
- *Capsule* shapes (using 2-vertex `PolygonShape` instances).
- More stable polygon stacking.
- Shared shapes for reduced memory usage.
- Support for C++11's range-based loops and constant expressions.
- Unit tested (via Google Test).
- Compile-time support for `double` and `long double` floating-point types and 32-bit and 64-bit fixed-point types (in addition to `float`).
- Fully per-step run-time configurable (via [`StepConf`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/StepConf.hpp)).
- In-depth per-step return value statistics (via [`StepStats`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/World.hpp#L86)).
- Increased construction-time configurability of world instances (via [`World::Def`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/World.hpp#L107)).

##About

Box2D is a 2D physics engine for games.

For help with Box2D, please visit http://www.box2d.org. There is a forum there where you may post your questions.

Please see Building.txt to learn how to build Box2D and run the testbed.

To run the demos, set "Testbed" as your startup project and press F5. Some test bed commands are:
- 'r' to reset the current test
- SPACE to launch a bomb
- arrow keys to pan
- 'x' and 'z' to zoom in/out
- use the mouse to click and drag objects

The main Box2D library is open source, but not open contribution. Please do not submit pull requests. Exceptions may be considered for build files and other items that are not part of the main library source. Thank you for your understanding.

Erin Catto
http://www.box2d.org

