# Unit Tests Console Application

This application component provides unit testing of the PlayRho library code.
It uses [Google's C++ test framework](https://github.com/google/googletest).

## Prerequisites

This application currently relies on the `UnitTests/googletest` git sub module,
so it has no prerequisites other than those for the [library component](../PlayRho/).

## Configuration

This component needs the `PLAYRHO_BUILD_UNIT_TESTS` CMake option turned on.
This can be achieved through the command line interface by adding the
`-DPLAYRHO_BUILD_UNIT_TESTS=ON` argument to the CMake configuration step's list
of arguments.

## Building & Installation

See the project's [documented build and installation steps](../INSTALL.md).
