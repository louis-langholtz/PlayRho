# Documentation

This directory contains documentation, configuration, and other resources for documentation.

Here's a listing of these documentation and generated documentation resources:

- [Online copy of the generated API docs](http://louis-langholtz.github.io/PlayRho/API/index.html).
- [The source code style guide for the project](StyleGuide.md).
- [Physical units interface insight](PhysicalUnits.md).
- [Collision handling differences from Box2D 2.3.2](CollisionHandlng.md).

Application Programming Interface documentation (API docs) can be manually generated from the [library source code](../Library) from some of this configuration using the [Doxygen](https://www.doxygen.nl) tool.
Using CMake, this can be done using steps like these:
```sh
cmake -S PlayRho -B PlayRhoBuild -DCMAKE_BUILD_TYPE=Release -DPLAYRHO_BUILD_DOC=ON -DPLAYRHO_BUILD_LIBRARY=OFF
cmake --build PlayRhoBuild --config Release
cmake --install PlayRhoBuild --prefix PlayRhoInstall
```
This assumes you're in the parent directory of the `PlayRho` directory,
that you're specifically interested in a `Release` build,
that you just want to build the API documentation,
that you want to build things in a separate directory called `PlayRhoBuild`,
and that you want to install the documentation under the `PlayRhoInstall` directory.
This is also done automatically as part of GitHub's continuous integration workflow system (see [docs.yml](../.github/workflows/docs.yml) for details).
Alternatively, see the top-level [building & installation](../INSTALL.md) document.
