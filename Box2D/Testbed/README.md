# Testbed GUI Application

The Testbed GUI application provides a GUI interface to a bunch of visually
presented demos. These may be helpful for testing and/or for learning how to use
[this fork of the Box2D library](https://github.com/louis-langholtz/Box2D).

Here in this directory, are two sub-directories. One for code for providing a
framework for the Testbed GUI application, and the other for demos. These demos
all subclass the [Test](Framework/Test.hpp) base class and would be where you'd
add your own code if you wanted it to also run under the Testbed GUI
application.

For more specifics, see the relevant directory.

## Build Instructions

Building the *Testbed* requires some library dependencies to be available:
the project library, the GLFW version 3 library, and the GLEW version 2 library.

### GLFW

[GLFW](http://www.glfw.org) is an Open Source library that the Testbed depends
on. The source code for this library is no longer included in this fork of
the project. The GLFW version 3 library must be available in an installed form
in order to build the Testbed. See the [GLFW Website](http://www.glfw.org) for
up-to-date information on it including how to download and install it.

Here are steps that can be used to download, build, and install this library
from source code using a command line environment:
1. Decide where you're going to have your GLFW sources. Setup an environment
   variable called `GLFW_SOURCES` that's set to this path. For example:
     `GLFW_SOURCES=/tmp/glfw-3.2.1`.
2. Download the GLFW 3.2.1 sources from
   https://github.com/glfw/glfw/releases/download/3.2.1/glfw-3.2.1.zip. Then
   extract them into `$GLFW_SOURCES`: `unzip -d /tmp glfw-3.2.1.zip`.
3. Run `cmake` on these sources. Maybe disable `GLFW_BUILD_DOCS` (and a few
   others).
4. Run `make`.
5. Run `make install`

### GLEW

[GLEW](http://glew.sourceforge.net) is an Open Source library that the Testbed
depends on. The source code for this library is no longer included in this fork
of the project. The GLEW version 2 library must be available in an installed
form in order to build the Testbed. See the
[GLEW Website](http://glew.sourceforge.net) for up-to-date information on it
including how to download and install it.

Here are steps that can be used to download, build, and install this library
from source code using a command line environment:
1. Get GLEW 2.
  - From a ZIP file:
    1. Download GLEW 2.0.0 from
       https://sourceforge.net/projects/glew/files/glew/2.0.0/glew-2.0.0.zip.
    2. Extract the sources: `unzip glew-2.0.0.zip`
  - As a git clone: `git clone https://github.com/nigels-com/glew.git glew`
2. Make sure to have the needed build tools.
  - For Debian/Ubuntu/Mint platforms: `$ sudo apt-get install build-essential libxmu-dev libxi-dev libgl-dev libosmesa-dev git`.
  - For RedHat/CentOS/Fedora:  `$ sudo yum install libXmu-devel libXi-devel libGL-devel git`
3. Run `make` and `make install` from within the source directory.
   It's not necessary to run `cmake` and the GLEW build instructions themselves
   don't say to run it.

### Testbed

When the GLFW 3 and GLEW 2 libraries are installed along with any specific
dependencies that they might have for your platform, the Testbed can be built.
The Testbed GUI application is currently building for Linux using CMake
and for Mac OS X using Xcode. With a little more work, the Testbed should be
able to be built for Windows.

#### Linux Using CMake

Assuming that GLFW and GLEW are installed under `/usr/local`:

    cd ${BUILD_DIR} && cmake -DBOX2D_BUILD_TESTBED=ON ${CMAKEFILE} && make

See the [Travis-CI configuration file](../../.travis.yml) for how this has
been done on the Linux CI build host.

#### Mac OS X Using Xcode

Assuming GLFW and GLEW are installed under `/usr/local`, double-click on
the Xcode project file from the Finder. Then select the `Testbed` scheme
and run it by pressing <kbd>&#8984;</kbd> <kbd>R</kbd> key combination.
This will first build the Testbed (and then it will run it).

#### Mac OS X Using CMake

TODO.

#### Windows Using VS2017 and the Solution File

TODO.

#### Windows Using VS2017 and CMake

TODO.

## Usage Instructions

To run the demos under MS Visual Studio, set `Testbed` as your startup project and press <kbd>F5</kbd>.

Once the Testbed is up and running, here are some keyboard and mouse commands
that can be used:
- <kbd>r</kbd> to reset the current test.
- <kbd>SPACE</kbd> to launch a bomb.
- <kbd>&larr;</kbd> <kbd>&rarr;</kbd> keys to pan.
- <kbd>x</kbd> and <kbd>z</kbd> to zoom in/out.
- <kbd>HOME</kbd> to reset the view (from zooming in/out).
- <kbd>[</kbd> and <kbd>]</kbd> to go to previous or next test.
- <kbd>P</kbd> to toggle pausing the test.
- <kbd>TAB</kbd> to toggle the appearance of the UI menu.
- Use the mouse to click and drag objects.
- <kbd>ESC</kbd> to exit.
