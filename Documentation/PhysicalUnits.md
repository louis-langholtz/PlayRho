# The Physical Units Interface

The *Physical Units Interface* for
[PlayRho](https://github.com/louis-langholtz/PlayRho), is an interface
that ties physical values to their required types.

The base implementation for the *physical units interface* simply uses
the `Real` type. To get the interface to enforce strong typing, a `constexpr`
enhanced implementation like the Boost 1.75.0 Units library has to be used.
Source code for that can be downloaded from the boost website at
https://www.boost.org.

To build with Boost units support:
  1. Download the latest Boost sources from boost.org. At the time this
     document was last updated, the latest was version 1.75.0. This can be
     downloaded from: https://www.boost.org/users/history/version_1_75_0.html.
  2. Extract these sources into a directory/folder of your choice.
     For the sake of example, let's say you chose `/usr/local`. So that after the extraction,
     there's now a sub-directory named `boost_1_75_0` in there.
  3. Add the path to that directory to your build system's list of include paths.
     If using the example directory, then you'd add `/usr/local/boost_1_75_0` to your
     build system's include paths.
  4. Go into your PlayRho project setup and add "PLAYRHO_USE_BOOST_UNITS" as a
     C pre-processor define so that the file
     [`PlayRho/Common/Units.hpp`](../PlayRho/Common/Units.hpp) can see that it's defined.
     This could be achieved just by uncommenting the line that appears like:
     `// #define PLAYRHO_USE_BOOST_UNITS`.
  5. Rebuild the PlayRho library and any applications. If you've used the
     physical units interface correctly everywhere, you should get no units
     related warnings or errors.
