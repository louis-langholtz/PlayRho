##What's New

###Status

|iOS|Linux|Mac OS X|Win32|
|---|-----|--------|-----|
|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=iosfoo)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=dev)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=macosxfoo)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=win32foo)](https://travis-ci.org/louis-langholtz/Box2D)|

###Overview

This is a fork by Louis Langholtz of the Box2D physics engine.

General influences on this fork are:
- [C++14](https://en.wikipedia.org/wiki/C%2B%2B14);
- [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md);
- [Unit testing](https://en.wikipedia.org/wiki/Unit_testing);
 
**NOTE**: These sources make heavy use of the standard C++ library [`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to confirm code correctness.
As such, this library *must* be compiled with the `NDEBUG` preprocessor macro enabled
to see any kind of performance.

###Changes

Here's a run-down of some of the changes:
- Exported symbols are all within the library namespace of `box2d` (and no longer preficed by `b2`).
- Preprocessor defines except those used for include guards, have been replaced with C++ solutions or removed from the API.
- Rounded corner collisions.
- *Capsule* shapes (using 2-vertex `PolygonShape` instances).
- More stable polygon stacking.
- Shared shapes for reduced memory usage.
- Support for C++11's range-based loops and constant expressions.
- Unit tested (via Google Test).
- Compile-time support for `double` and `long double` floating-point types and 32-bit and 64-bit fixed-point types (in addition to `float`).
- Fully per-step run-time configurable (via [`StepConf`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/StepConf.hpp)).
- In-depth per-step return value statistics (via [`StepStats`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/StepStats.hpp)).
- Increased construction-time configurability of world instances (via [`World::Def`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Box2D/Dynamics/World.hpp#L107)).
- Various methods have been rewritten to be [non-member non-friend functions](http://www.drdobbs.com/cpp/how-non-member-functions-improve-encapsu/184401197).
- Various functions and procedures have been rewritten to be ["pure functions"](https://en.wikipedia.org/wiki/Pure_function).
- Testbed enhancements: provides controls for per-step configurability, displays per-step statistics, ability to manipulate bodies while paused, and more.

##About

Box2D is a 2D physics engine [licensed](https://github.com/louis-langholtz/Box2D/blob/dev/LICENSE.txt) under a [Zlib License](https://opensource.org/licenses/Zlib).

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

