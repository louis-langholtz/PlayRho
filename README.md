# PlayRho

*A way to play with physical behaviors like the conservation of momentum.*

PlayRho is a real-time oriented physics engine and library that's currently best suited for
2-D games. The project name is the composition of the verb *play* with the noun *rho* where
*rho* is the Greek letter often used to represent physical quantities like momentum.

## Background

This is a fork by [Louis Langholtz](https://github.com/louis-langholtz)
of the Box2D 2.3.2 physics engine originally written by Erin Catto. It's
[licensed](LICENSE.txt) under a [Zlib License](https://opensource.org/licenses/Zlib).

General goals of this project are:
- Using and supporting the newer features of the C++ language standards, particularly [C++14](https://en.wikipedia.org/wiki/C%2B%2B14);
- Conforming to the [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md),
  particularly in regards to things like avoiding global variables,
  preferring initialization, and preferring pure functions;
- A greater openness to contributions including pull requests with new features; and
- Providing a [Continuous Integration](https://en.wikipedia.org/wiki/Continuous_integration)
  backed, [Build Automation](https://en.wikipedia.org/wiki/Build_automation) supported, and
  [Unit Tested](https://en.wikipedia.org/wiki/Unit_testing) project.

**NOTE**: These sources make heavy use of the standard C++ library
[`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to help confirm
correctness of the code logic. As such, this library *must* be compiled with
the `NDEBUG` pre-processor macro enabled, and of course optimization turned on,
to see any kind of performance.

## Continuous Integration Status

|iOS|Linux CMake|macOS Xcode|Windows VS2017|Coverage|
|---|-----------|-----------|--------------|--------|
|[![Build Status](https://travis-ci.org/louis-langholtz/PlayRho.svg?branch=iosfoo)](https://travis-ci.org/louis-langholtz/PlayRho)|[![Build Status](https://travis-ci.org/louis-langholtz/PlayRho.svg?branch=master)](https://travis-ci.org/louis-langholtz/PlayRho)|[![Build Status](https://travis-ci.org/louis-langholtz/PlayRho.svg?branch=macosxfoo)](https://travis-ci.org/louis-langholtz/PlayRho)|[![Build status](https://ci.appveyor.com/api/projects/status/buoix7kumafgsjtu/branch/master?svg=true)](https://ci.appveyor.com/project/louis-langholtz/playrho/branch/master)|[![Coverage Status](https://coveralls.io/repos/github/louis-langholtz/PlayRho/badge.svg?branch=master)](https://coveralls.io/github/louis-langholtz/PlayRho?branch=master)|

## Additional Resources

- [Changes Document](Changes.md):
  provides a run-down of changes between PlayRho and Box2D 2.3.2.
- [Building Document](INSTALL.md):
  information on how to build this project and run the Testbed.
- [API Pages](http://louis-langholtz.github.io/PlayRho/API/index.html): documentation on the Application Programming Interface (API).
- [Issues Web Interface](https://github.com/louis-langholtz/PlayRho/issues):
  for questions, bugs reports, or suggestions associated with this project.
