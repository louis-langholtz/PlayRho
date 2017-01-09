##What's New

This is a fork by Louis Langholtz of the Box2D physics engine.

Particular influences on this fork are:
- [Unit testing](https://en.wikipedia.org/wiki/Unit_testing);
- [C++11](https://en.wikipedia.org/wiki/C%2B%2B11) & [C++14](https://en.wikipedia.org/wiki/C%2B%2B14);
- [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md);
- Effective Modern C++ by Scott Meyers; and
- Hardware capabilities of popular CPU architectures like the
  [Intel-64 architecture](http://www.intel.com/content/dam/www/public/us/en/documents/manuals/64-ia-32-architectures-software-developer-manual-325462.pdf).
 
**NOTE**: These sources make heavy use of the standard C++ library [`std::assert`](http://en.cppreference.com/w/cpp/error/assert) macro to confirm code correctness.
As such, this library *must* be compiled with the `NDEBUG` preprocessor macro enabled
to see any kind of performance.

##About

Box2D is a 2D physics engine for games.

For help with Box2D, please visit http://www.box2d.org. There is a forum there where you may post your questions.

Please see Building.txt to learn how to build Box2D and run the testbed.

To run the demos, set "Testbed" as your startup project and press F5. Some test bed commands are:
- 'r' to reset the current test
- SPACE to launch a bomb
- arrow keys to pan
- 'x' and 'z' to zoom in/out
- use the mouse to click and drag objects

The main Box2D library is open source, but not open contribution. Please do not submit pull requests. Exceptions may be considered for build files and other items that are not part of the main library source. Thank you for your understanding.

Erin Catto
http://www.box2d.org

