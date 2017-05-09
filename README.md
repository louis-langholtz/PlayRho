## About

This is a fork by [Louis Langholtz](https://github.com/louis-langholtz)
of the Box2D physics engine originally written by Erin Catto. It's
[licensed](https://github.com/louis-langholtz/Box2D/blob/dev/LICENSE.txt) under
a [Zlib License](https://opensource.org/licenses/Zlib). For the original Box2D, please visit http://www.box2d.org.

General influences on this fork are:
- [C++14](https://en.wikipedia.org/wiki/C%2B%2B14);
- [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md);
- [Unit Testing](https://en.wikipedia.org/wiki/Unit_testing);

**NOTE**: These sources make heavy use of the standard C++ library
[`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to help confirm
correctness of the code logic. As such, this library *must* be compiled with
the `NDEBUG` preprocessor macro enabled to see any kind of performance.

## Continuous Integration Status

|iOS|Linux CMake|OS X Xcode|Windows VS2017|
|---|-----------|----------|--------------|
|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=iosfoo)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=dev)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build Status](https://travis-ci.org/louis-langholtz/Box2D.svg?branch=macosxfoo)](https://travis-ci.org/louis-langholtz/Box2D)|[![Build status](https://ci.appveyor.com/api/projects/status/q4q1y1g6ckqqpiev/branch/dev?svg=true)](https://ci.appveyor.com/project/louis-langholtz/box2d/branch/dev)|

## Additional Resources

- [Changes Document](https://github.com/louis-langholtz/Box2D/blob/dev/Changes.md):
  provides a run-down of changes that have been introduced.
- [Building Document](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Building.txt):
  information on how to build this project and run the testbed.
- [API Pages](http://louis-langholtz.github.io/Box2D/API/index.html): documentation on the Application Programming Interface (API).
- [Issues Web Interface](https://github.com/louis-langholtz/Box2D/issues):
  for questions, bugs reports, or suggestions associated with this project.
