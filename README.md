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
[![latest packaged version(s)](https://repology.org/badge/latest-versions/playrho.svg?header=latest%20packaged)](https://repology.org/project/playrho/versions)
[![Ask questions at StackOverflow with the tag playrho](https://img.shields.io/badge/stackoverflow-playrho-blue.svg)](https://stackoverflow.com/questions/tagged/playrho)

*A way to play with physical behaviors like the conservation of momentum.*

## Overview

PlayRho is a real-time oriented, platform independent, physics engine and library.
It's currently best suited for interactive 2-D games or demos.
The project's name is the composition of the verb *play* with the noun *rho*, where *rho* is the Greek letter often used to represent physical quantities like momentum.

PlayRho started off as a port by Louis Langholtz of the Box2D 2.3.2 physics engine to ["modern C++"](https://msdn.microsoft.com/en-us/library/hh279654.aspx).
It's evolved into a derivative work by Louis and other contributors to the code base.
Like its predecessor, PlayRho is also [licensed](LICENSE.txt) under a [Zlib License](https://opensource.org/licenses/Zlib).
Many other open source physics engines exist, like: [Bullet Physics](http://bulletphysics.org/) and [Chipmunk](https://chipmunk-physics.net).

The PlayRho library component itself requires only a standards compliant C++17 compiler and standard library implementation.
It's continuous integration backed and unit test proven to compile and work on at least Linux, macOS, and Windows.
See the status badges above for up-to-date status of builds, tests, documentation, code-security, and more.

By design, new development is done in the default/master branch, merged in by pull requests, and then possibly backported to a release branch if not API breaking.
While the master branch is intended to always be buildable and runnable, its interface is not meant to be stable and it's not meant for use unless you're specifically looking to help develop this project.
For use in projects, choose from a more stable [tagged release](https://github.com/louis-langholtz/PlayRho/releases) or a release branch.

## General Goals

- Supporting value semantics over reference semantics. Values are in and pointers are out!
- Conforming to the [C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines), particularly in regards to things like: [avoiding non-constant global variables](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Ri-global), [preferring concrete types](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rc-concrete), and [preferring pure functions](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rf-pure).
- Using and supporting newer features of the C++ language standards up to [C++17](https://en.wikipedia.org/wiki/C%2B%2B17).
- Openness to contributions including pull requests with new features.
- Providing a [continuous integration](https://en.wikipedia.org/wiki/Continuous_integration) backed, [build automation](https://en.wikipedia.org/wiki/Build_automation) supported, [unit test](https://en.wikipedia.org/wiki/Unit_testing) validated, and [benchmark](https://en.wikipedia.org/wiki/Benchmark_(computing)) assessed, project.

## Additional Resources:

- [Changes Document](Changes.md): provides a run-down of changes between releases.
- [Building & Installation Document](INSTALL.md): information on how to build and install this project.
- [API Documentation](http://louis-langholtz.github.io/PlayRho/API/index.html): application programming interface (API) pages for this project.
- [Issues Web Interface](https://github.com/louis-langholtz/PlayRho/issues): for bugs reports or feature requests associated with this project.
- [stackoverflow.com playrho tag](https://stackoverflow.com/questions/tagged/playrho): for questions and answers on this project.
