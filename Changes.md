# Changes

Here's a run-down of some of the changes
[this fork of Box2D](https://github.com/louis-langholtz/Box2D) introduces categorized
as user-level changes or internal changes.

## User-Level Changes

- Exported symbols are now within the library namespace of `box2d` and are no
  longer prefaced by `b2`.
- Mutable global variables in the library have been removed or replaced with
  runtime-time parameters.
- Preprocessor defines, except those used for include guards, have been
  replaced with C++ solutions or removed from the API.
- All shapes can now be used with dynamic bodies (support for point selection
  and associated mass data information has been added for chain and edge shapes).
- All shapes can collide with any other shape now (chains can collide with other
  chains or edges or any other kind of shape).
- All shapes support vertex-radius based corner rounding now. This enables
  [*capsule shapes* and other visibly rounded shapes](https://github.com/louis-langholtz/Box2D/blob/dev/Documentation/images/RoundedCornerShapes.png).
- [Rounded and modified corner collisions](https://github.com/louis-langholtz/Box2D/blob/dev/Documentation/CollisionHandlng.md).
  This supports the physical behaviors expected of rounded corners, like
  increased roll-ability, while preventing dragged shapes from getting stuck
  on composite surfaces.
- Vertex-radius respecting `RayCast` functionality for all shapes now
  (`RayCast` functionality had been only respecting the vertex-radius for
  circle shapes).
- Support for up to 254-vertex polygons.
- More stable polygon stacking.
- Shared shape ownership, with friction, density, and restitution moved into
  them (from Fixture class) for reduced memory usage.
- Circle shapes have been renamed as "disk" shapes to distinguish them as solid and never hollow.
- Support for C++11's range-based loops and constant expressions.
- Compile-time support for zero-runtime strongly-typed physical units (using an
  interface to [`constexpr`-enhanced boost units](https://github.com/louis-langholtz/units)).
  For details on how to enable this, see
  [Documentation/PhysicalUnits.md](https://github.com/louis-langholtz/Box2D/blob/dev/Documentation/PhysicalUnits.md).
- Compile-time support for `double` and `long double` floating-point types, and
  32-bit and 64-bit fixed-point types (in addition to `float`).
- Fully per-step run-time configurable (via
  [`StepConf`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Dynamics/StepConf.hpp)).
- In-depth per-step return value statistics (via
  [`StepStats`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Dynamics/StepStats.hpp)).
- Increased construction-time configurability of world instances (via
  [`World::Def`](https://github.com/louis-langholtz/Box2D/blob/dev/Box2D/Dynamics/World.hpp#L107)).
- Various methods have been rewritten to be
  [non-member non-friend functions](http://www.drdobbs.com/cpp/how-non-member-functions-improve-encapsu/184401197).
- Various functions and procedures have been rewritten to be
  ["pure functions"](https://en.wikipedia.org/wiki/Pure_function).
- World locked related asserts have been replaced by throws of `World::LockedError`
  instances.
- Testbed enhancements: per-step configurability, per-step statistics, ability
  to manipulate bodies while paused, additional paths to find font, a font-less
  mode (instead of aborting), and more.
- Testbed test additions: Half Pipe, iforce2d's Topdown Car, Orbiter, Newton's
  Cradle, and Spinning Circles.
- Adds online [API Documentation](http://louis-langholtz.github.io/Box2D/API/index.html).

## Internal Changes

- Unit tested via [Google Test](https://github.com/google/googletest/tree/aa148eb2b7f70ede0eb10de34b6254826bfb34f4)
  and [over 400 tests](https://github.com/louis-langholtz/Box2D/tree/dev/UnitTests).
- Continuous integration (CI) building and unit testing of repository updates
  for the Linux and Windows platforms.
- The [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md)
  have been accepted as the standard guidance for new and modified source code.
- Tab spacing has all been replaced by four space-character spacing.
- Custom memory allocators replaced by standard library allocation methods.
- Most structures and classes are now in their own separate files.
- The library now uses exceptions and if built with exception support enabled can allow users
  to intercept otherwise fatal situations such as invoking a method that's only allowed to be
  called while the world is not locked.
