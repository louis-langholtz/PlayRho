# HelloWorld Console Application

This application component provides a time-wise dump of a body's `X`-`Y` coordinates and angle after being dropped from a distance over a surface.
Output is limited to the preconfigured number of steps of the engine, the acceleration towards the surface, and the distance from that surface.
Only the `Y` coordinate value should change from line to line until the number of steps have finished or the body has come to rest on the surface,
which ever may come first.

## Prerequisites

This component has no prerequisites other than those for the [library component](../PlayRho/).

## Configuration

This component needs the `-DPLAYRHO_BUILD_HELLOWORLD=ON` command line argument added to the CMake configuration step's list of arguments.

## Building & Installation

See the project's [documented build and installation steps](../INSTALL.md).

## Sample Output

```sh
$ ./HelloWorld
 0.00 4.00 0.00
 0.00 3.99 0.00
 0.00 3.98 0.00
 0.00 3.97 0.00
 0.00 3.96 0.00
 0.00 3.94 0.00
 0.00 3.92 0.00
 0.00 3.90 0.00
 0.00 3.88 0.00
 0.00 3.85 0.00
 0.00 3.82 0.00
 0.00 3.79 0.00
 0.00 3.75 0.00
 0.00 3.71 0.00
 0.00 3.67 0.00
 0.00 3.63 0.00
 0.00 3.58 0.00
 0.00 3.53 0.00
 0.00 3.48 0.00
 0.00 3.43 0.00
 0.00 3.37 0.00
 0.00 3.31 0.00
 0.00 3.25 0.00
 0.00 3.18 0.00
 0.00 3.12 0.00
 0.00 3.04 0.00
 0.00 2.97 0.00
 0.00 2.89 0.00
 0.00 2.82 0.00
 0.00 2.73 0.00
 0.00 2.65 0.00
 0.00 2.56 0.00
 0.00 2.47 0.00
 0.00 2.38 0.00
 0.00 2.28 0.00
 0.00 2.19 0.00
 0.00 2.09 0.00
 0.00 1.98 0.00
 0.00 1.88 0.00
 0.00 1.77 0.00
 0.00 1.66 0.00
 0.00 1.54 0.00
 0.00 1.42 0.00
 0.00 1.30 0.00
 0.00 1.18 0.00
 0.00 1.06 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
 0.00 1.00 0.00
```
