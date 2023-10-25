# Changes

## Release 1.1 To Release 2.0

For a complete detailed view of changes from a git-based source code perspective between release 1.1 and release 2.0, see:

```
git diff release-1.1..master
```

Here's a writeup of these changes:

- Clang-tidy uses more checks.
- `PlayRho/PlayRho` has become `PlayRho/Library`.
- Platform target workflows now test CMake install and produce artifacts.
- Some CMake options have become dependent options.
- Fixtures have been absorbed by shapes.
- The library has been reorganized in terms of namespaces and header versus source files, instead of by functional category.
- Many details have been eliminated or moved under a "detail" sub-namespace to de-clutter the user API.
- Declarations & definitions of 2-dimensional code have moved under the "playrho::d2" sub-namespace, and apart from dimensionless code.
- Many "magic-values" have been replaced by named constants.
- Many interfaces now take more general Span parameters instead of their specific container types.
- `CheckedValue` has become `Checked`.
- The checked value class template's design has been updated to optionally also support fail fast instantiations and to more consistently utilize checker types.
- New checker types have been added & made use of to better recognize variables and their uses.
- More variables now use checked types to more tightly constrain & better document their usage.
- Custom memory allocation via C++ allocators has been incorporated into the implementation of "hotter" code paths of the world stepping functionality.
- More custom types are now defined in their own include and source files.
- The `Testbed` application has been updated to support significantly more user modification of demos at runtime.
- The `Fixed` class template and related code has been moved out from this project and into my separate [realnumb](https://github.com/louis-langholtz/realnumb) repository and project.
- CMake configuration now supports customization of some values in `Settings.hpp` via the following variables:
  - `PLAYRHO_REAL_LINEARSLOP`: overriding `DefaultLinearSlop`.
  - `PLAYRHO_REAL_MINVERTEXRADIUS`: overriding `DefaultMinVertexRadius`.
  - `PLAYRHO_REAL_MAXVERTEXRADIUS`: overriding `DefaultMaxVertexRadius`.
- CMake configuration for the `Real` alias defined in `Real.hpp` now supports all of the following variables:
  - `PLAYRHO_REAL_TYPE`: to override the type of the `Real` alias from `float`. Can be `float`, `double`, `long double`, or a custom type, `T`, with similar support. See `Real.hpp` for more details.
  - `PLAYRHO_REAL_PROJECT`: to specify a package/project name providing support for a custom `Real` type.
  - `PLAYRHO_REAL_INCLUDE`: to add one or more semicolon separated lines to the `Real.hpp` file for doing things like adding includes necessary for a custom `Real` type.
  - `PLAYRHO_REAL_GIT_REPOSITORY`: if `PLAYRHO_REAL_PROJECT` is set, this optionally specifies a git repository from where the project is to be CMake fetched from.
  - `PLAYRHO_REAL_GIT_TAG`: if `PLAYRHO_REAL_GIT_REPOSITORY` is set, then this specifies the `GIT_TAG` of the repository to fetch.

## Box2D 2.3.2 To Release 1.1

Here's a run-down of some of the changes in going from Box2D 2.3.2 to
[PlayRho Release 1.1](https://github.com/louis-langholtz/PlayRho/tree/release-1.1), categorized
as API (Application Programming Interface) changes, Testbed changes, and other changes.

### API Changes

- Stripped the `b2` prefix off of names.
- Moved exported symbols into the namespace of `playrho`.
- Moved exported symbols related to 2-dimensional space into the nested namespace of `playrho::d2`.
- Eliminated all mutable global variables.
- Replaced pre-processor defines, except those used for include guards, with C++ solutions.
- Unified dynamic body support so all shapes can now be used with dynamic bodies.
  Support for point selection and associated mass data information has been added for
  chain and edge shapes.
- Completed collision support for all shapes.
  All shapes can collide with any other shape now (chains can collide with other
  chains or edges or any other kind of shape).
- All shapes support vertex-radius based corner rounding now. This enables
  [*capsule shapes* and other visibly rounded shapes](Documentation/images/RoundedCornerShapes.png).
- [Rounded and modified corner collisions](Documentation/CollisionHandlng.md).
  This supports the physical behaviors expected of rounded corners, like
  increased roll-ability, while preventing dragged shapes from getting stuck
  on composite surfaces.
- Vertex-radius respecting `RayCast` functionality for all shapes now
  (`RayCast` functionality had been only respecting the vertex-radius for
  circle shapes).
- Refactored polygon shapes to support up to 254-vertices.
- More stable polygon stacking.
- Shared shape ownership, with friction, density, and restitution moved into
  them (from Fixture class) for reduced memory usage.
- Renamed circle shapes as "disk" shapes to distinguish them as solid and never hollow.
- Switched from reference-semantics to value-semantics. Pointers are gone from the interface. References to bodies, fixtures, joints, and contacts have been replaced with identifiers and function interfaces.
- Converted public "virtual" interfaces to value-semantic polymorphic value types. For an introduction to polymorphic value types, please check out [C++: My Love Affair With Polymorphic Value Types](https://gist.github.com/louis-langholtz/5da900c8333eed26641a09bea7aa5c31).
- Added support for C++11 range-based loops and constant expressions.
- Added compile-time support for "zero-runtime-overhead" strongly-typed physical units (using an
  interface to [`constexpr` enhanced boost units](https://github.com/louis-langholtz/units)).
  For details on how to enable this, see
  [Documentation/PhysicalUnits.md](Documentation/PhysicalUnits.md).
- Added compile-time support for `double` and `long double` floating-point types (in addition to `float`), and 32-bit and 64-bit fixed-point types.
- Fully per-step run-time configurable (via
  [`StepConf`](Library/include/PlayRho/Dynamics/StepConf.hpp)).
- In-depth per-step return value statistics (via
  [`StepStats`](Library/include/PlayRho/Dynamics/StepStats.hpp)).
- Increased construction-time configurability of world instances (via
  [`WorldConf`](Library/include/PlayRho/Dynamics/WorldConf.hpp)).
- Refactored numerous methods to be
  [non-member non-friend functions](http://www.drdobbs.com/cpp/how-non-member-functions-improve-encapsu/184401197).
- Refactored various functions and procedures to be
  ["pure functions"](https://en.wikipedia.org/wiki/Pure_function).
- Replaced world locked related asserts with throws of `World::LockedError`
  instances.
- Removed the gravity concept from `World` instances in favor of increasing performance of steps by consolidating acceleration entirely within bodies.
- Added many per-step run-time configuration capabilities.
- Increased per-step statistics.
- Added online [API Documentation](http://louis-langholtz.github.io/PlayRho/API/index.html).

### Testbed changes

- Updated UI to provide full access to the per-step configurability and per-step statistics.
- Added the ability to manipulate bodies while paused.
- Added additional paths to find font and a font-less mode (instead of aborting).
- Added more demonstrations/tests: Half Pipe, iforce2d's Topdown Car, Orbiter, Newton's
  Cradle, Spinning Circles, and Solar System.
- Added the ability to read and update shapes, bodies, joints, and contacts (since PR#414 from the master branch and coming up in release 2).
- Added the ability to create and destroy shapes, bodies, and joints (since PR#414 from the master branch and coming up in release 2).
- Numerous other changes.

### Other Changes

- Unit tested via [Google Test](https://github.com/google/googletest/tree/aa148eb2b7f70ede0eb10de34b6254826bfb34f4)
  and [over 1100 tests](UnitTests).
- Added continuous integration (CI) building and unit testing of repository updates
  for the Linux and Windows platforms.
- Configured use of [COVERALLS](https://coveralls.io/github/louis-langholtz/PlayRho?branch=dev)
  for CI unit test code coverage analysis.
- Accepted the [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md)
  as the standard guidance for new and modified source code.
- Replaced all tab spacing with four space-character spacing.
- Replaced custom memory allocators with standard library allocation methods.
- Separated most structures and classes out into their own separate files.
- The library now uses exceptions and if built with exception support enabled can allow users
  to intercept otherwise fatal situations such as invoking a method that's only allowed to be
  called while the world is not locked.
