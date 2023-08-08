<!--
  This is written for GitHub Flavored Markdown.
  See: https://github.github.com/gfm/
-->

# PlayRho

[![linux](https://github.com/louis-langholtz/PlayRho/actions/workflows/linux.yml/badge.svg)](https://github.com/louis-langholtz/PlayRho/actions/workflows/linux.yml)
[![macos](https://github.com/louis-langholtz/PlayRho/actions/workflows/macos.yml/badge.svg)](https://github.com/louis-langholtz/PlayRho/actions/workflows/macos.yml)
[![windows](https://github.com/louis-langholtz/PlayRho/actions/workflows/windows.yml/badge.svg)](https://github.com/louis-langholtz/PlayRho/actions/workflows/windows.yml)
[![API Documentation](https://github.com/louis-langholtz/PlayRho/actions/workflows/docs.yml/badge.svg)](https://github.com/louis-langholtz/PlayRho/actions/workflows/docs.yml)
[![CodeQL](https://github.com/louis-langholtz/PlayRho/actions/workflows/codeql.yml/badge.svg)](https://github.com/louis-langholtz/PlayRho/actions/workflows/codeql.yml)
[![Coverage Status](https://coveralls.io/repos/github/louis-langholtz/PlayRho/badge.svg?branch=master)](https://coveralls.io/github/louis-langholtz/PlayRho?branch=master)
[![Ask questions at StackOverflow with the tag playrho](https://img.shields.io/badge/stackoverflow-playrho-blue.svg)](https://stackoverflow.com/questions/tagged/playrho)

*A way to play with physical behaviors like the conservation of momentum.*

## Overview

PlayRho is a real-time oriented, platform independent, physics engine and library, that's currently best suited for interactive 2-D games or demos.
The project's name is the composition of the verb *play* with the noun *rho*, where
*rho* is the Greek letter often used to represent physical quantities like momentum.

PlayRho started off as a port by Louis Langholtz of the Box2D 2.3.2 physics engine to ["modern C++"](https://msdn.microsoft.com/en-us/library/hh279654.aspx).
It's evolved into a derivative work by Louis and other contributors to the code base.
Like its predecessor, PlayRho is also [licensed](LICENSE.txt) under a [Zlib License](https://opensource.org/licenses/Zlib).
Many other open source physics engines exist, like: [Bullet Physics](http://bulletphysics.org/) and [Chipmunk](https://chipmunk-physics.net).

The PlayRho library component itself requires only a standards compliant C++17 compiler and standard library implementation.
It's continuous integration backed and unit test proven to compile and work on at least Linux, macOS, and Windows.
See the status badges above for up-to-date status of builds, tests, documentation, code-security, and more.

By design, new development is done in the default/master branch, merged in by pull requests, and then possibly
backported to a release branch if not API breaking.
While the master branch is intended to always be buildable and runnable,
its interface is not meant to be stable and it's not meant for use unless you're specifically looking to help develop this project.
For use in projects,
choose from a more stable [tagged release](https://github.com/louis-langholtz/PlayRho/releases) or a release branch.

## General Goals

- Supporting value semantics over reference semantics. Values are in and pointers are out!
- Conforming to the [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md),
  particularly in regards to things like avoiding global variables,
  preferring concrete types, and preferring pure functions.
- Using and supporting newer features of the C++ language standards up to [C++17](https://en.wikipedia.org/wiki/C%2B%2B17);
- Openness to contributions including pull requests with new features; and
- Providing a [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration)
  backed, [build automation](https://en.wikipedia.org/wiki/Build_automation) supported,
  [unit test](https://en.wikipedia.org/wiki/Unit_testing) validated, and [benchmark](https://en.wikipedia.org/wiki/Benchmark_(computing)) assessed, project.

## Additional Resources:

- [Changes Document](Changes.md):
  provides a run-down of changes between PlayRho and Box2D 2.3.2.
- [Building & Installation Document](INSTALL.md):
  information on how to build this project and run the Testbed.
- [User Documentation & API Pages](http://louis-langholtz.github.io/PlayRho/API/index.html): user documentation and programming interface pages for [release 1.1.0](https://github.com/louis-langholtz/PlayRho/tree/v1.1.0) of this project.
- [Issues Web Interface](https://github.com/louis-langholtz/PlayRho/issues):
  for questions, bugs reports, or suggestions associated with this project.
