# PlayRho Library

This component is the base required component for the project.
None of the other components are necessary if you only want to build and install the library.

If you want to build any other components, see the other component folders' README.md files.

## Prerequisites

This component does not have any prerequisites other than the minimum project prerequisites.

## Configuration

This component doesn't need any configuration options to be specified.

## Building & Installation

See the project's [documented build and installation steps](../INSTALL.md).

## Layout

- [`include/playrho`](include/playrho): Files and subdirectories for the library's header files.
- [`source/playrho`](source/playrho): Files and subdirectories for the library's source files.

Contents under these directories are otherwise named and structured by functionality and C++ namespaces.
For example, code primarily for 2-dimensional use is under a `d2` subdirectory and within the `playrho::d2` namespace.

## History

This directory/folder used to be called `PlayRho`,
as a subdirectory of the project directory also of the same name,
i.e. `PlayRho/PlayRho`.
It's always been for the library component however.
Changing its name to `Library` avoids this repetition of the name,
and hopefully clarifies what it's for.

This component's files used to be split up into subdirectories by functionality - dynamics, collision, etc..
The files have since been rearranged to more closely match their primarily associated namespaces,
and header files have been separated from source files into the sub-directories of `include/playrho` and `source/playrho`.
Besides hopefully making it easier for users to understand the API,
this is also a more conventional layout for C++ libraries and more closely matches what the installed project's include directory looks like.
