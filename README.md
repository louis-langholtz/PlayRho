# PlayRho

*A way to play with physical behaviors like the conservation of momentum.*

PlayRho is a real-time oriented physics engine and library that's currently best suited for
2-D games. The project name is the composition of the verb *play* with the noun *rho* where
*rho* is the Greek letter often used to represent physical quantities like momentum.

## Background

PlayRho started off as a port by Louis Langholtz of the Box2D 2.3.2 physics engine to ["modern C++"](https://msdn.microsoft.com/en-us/library/hh279654.aspx). It's evolved into a derivative work by Louis and other contributors to the code base. Like its predecessor, PlayRho is also [licensed](LICENSE.txt) under a [Zlib License](https://opensource.org/licenses/Zlib).

Some general goals of this project are:
- Supporting value semantics over reference semantics. Values are in and pointers are out!
- Conforming to the [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md),
  particularly in regards to things like avoiding global variables,
  preferring concrete types, and preferring pure functions.
- Using and supporting newer features of the C++ language standards up to [C++17](https://en.wikipedia.org/wiki/C%2B%2B17);
- Openness to contributions including pull requests with new features; and
- Providing a [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration)
  backed, [build automation](https://en.wikipedia.org/wiki/Build_automation) supported,
  [unit testing](https://en.wikipedia.org/wiki/Unit_testing) validated, and [benchmark](https://en.wikipedia.org/wiki/Benchmark_(computing)) assessed, project.

## Continuous Integration Status

|iOS|Linux CMake|macOS Xcode|Windows VS2017|Coverage|CodeQL|
|---|-----------|-----------|--------------|--------|------|
|[![Build Status](https://travis-ci.org/louis-langholtz/PlayRho.svg?branch=iosfoo)](https://travis-ci.org/louis-langholtz/PlayRho)|[![Build Status](https://travis-ci.org/louis-langholtz/PlayRho.svg?branch=master)](https://travis-ci.org/louis-langholtz/PlayRho)|[![Build Status](https://travis-ci.org/louis-langholtz/PlayRho.svg?branch=macosxfoo)](https://travis-ci.org/louis-langholtz/PlayRho)|[![Build status](https://ci.appveyor.com/api/projects/status/buoix7kumafgsjtu/branch/master?svg=true)](https://ci.appveyor.com/project/louis-langholtz/playrho/branch/master)|[![Coverage Status](https://coveralls.io/repos/github/louis-langholtz/PlayRho/badge.svg?branch=master)](https://coveralls.io/github/louis-langholtz/PlayRho?branch=master)|![CodeQL](https://github.com/louis-langholtz/PlayRho/workflows/CodeQL/badge.svg)|

## Additional Resources

- [Changes Document](Changes.md):
  provides a run-down of changes between PlayRho and Box2D 2.3.2.
- [Building Document](INSTALL.md):
  information on how to build this project and run the Testbed.
- [API Pages](http://louis-langholtz.github.io/PlayRho/API/index.html): documentation on the Application Programming Interface (API).
- [Issues Web Interface](https://github.com/louis-langholtz/PlayRho/issues):
  for questions, bugs reports, or suggestions associated with this project.
