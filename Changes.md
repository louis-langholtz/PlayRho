# Changes

Here's a run-down of some of the changes between
[PlayRho](https://github.com/louis-langholtz/PlayRho) and Box2D 2.3.2, categorized
as Application Programming Interface (API) changes and non-API (other) changes.

## API Changes

- Moved exported symbols into the namespace of `playrho`.
- Stripped the `b2` prefix off of names.
- Eliminated all mutable global variables by replacing them with runtime-time parameters.
- Replaced preprocessor defines, except those used for include guards, with C++ solutions.
- Unified dynamic body support so all shapes can now be used with dynamic bodies.
  Support for point selection and associated mass data information has been added for
  chain and edge shapes.
- Completed collision support for all shapes.
  All shapes can collide with any other shape now (chains can collide with other
  chains or edges or any other kind of shape).
- All shapes support vertex-radius based corner rounding now. This enables
  [*capsule shapes* and other visibly rounded shapes](https://github.com/louis-langholtz/PlayRho/blob/dev/Documentation/images/RoundedCornerShapes.png).
- [Rounded and modified corner collisions](https://github.com/louis-langholtz/PlayRho/blob/dev/Documentation/CollisionHandlng.md).
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
- Added support for C++11's range-based loops and constant expressions.
- Added compile-time support for zero-runtime strongly-typed physical units (using an
  interface to [`constexpr`-enhanced boost units](https://github.com/louis-langholtz/units)).
  For details on how to enable this, see
  [Documentation/PhysicalUnits.md](https://github.com/louis-langholtz/PlayRho/blob/dev/Documentation/PhysicalUnits.md).
- Added compile-time support for `double` and `long double` floating-point types, and
  32-bit and 64-bit fixed-point types (in addition to `float`).
- Fully per-step run-time configurable (via
  [`StepConf`](https://github.com/louis-langholtz/PlayRho/blob/dev/PlayRho/Dynamics/StepConf.hpp)).
- In-depth per-step return value statistics (via
  [`StepStats`](https://github.com/louis-langholtz/PlayRho/blob/dev/PlayRho/Dynamics/StepStats.hpp)).
- Increased construction-time configurability of world instances (via
  [`World::Def`](https://github.com/louis-langholtz/PlayRho/blob/dev/PlayRho/Dynamics/World.hpp#L107)).
- Refactored various methods to be
  [non-member non-friend functions](http://www.drdobbs.com/cpp/how-non-member-functions-improve-encapsu/184401197).
- Refactored various functions and procedures to be
  ["pure functions"](https://en.wikipedia.org/wiki/Pure_function).
- Replaced world locked related asserts with throws of `World::LockedError`
  instances.
- Testbed enhancements: per-step configurability, per-step statistics, ability
  to manipulate bodies while paused, additional paths to find font, a font-less
  mode (instead of aborting), and more.
- Testbed test additions: Half Pipe, iforce2d's Topdown Car, Orbiter, Newton's
  Cradle, and Spinning Circles.
- Added online [API Documentation](http://louis-langholtz.github.io/PlayRho/API/index.html).

## Other Changes

- Unit tested via [Google Test](https://github.com/google/googletest/tree/aa148eb2b7f70ede0eb10de34b6254826bfb34f4)
  and [over 400 tests](https://github.com/louis-langholtz/PlayRho/tree/dev/UnitTests).
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
