# Testbed GUI Application

The Testbed GUI application provides a GUI interface to a bunch of visually
presented demos. These may be helpful for testing and/or for learning how to use
[PlayRho](https://github.com/louis-langholtz/PlayRho).

Here in this directory, are three sub-directories:
1. [`Data`](Data/): For data for the application.
2. [`Framework`](Framework/): For code providing a framework for the
   application like the [Test](Framework/Test.hpp) base class.
3. [`Tests`](Tests/): Containing code for demos. These demos all subclass the
   `Test` base class. This folder is where you'd add your own code if you
   wanted it to also run under the Testbed GUI application.

For more specifics, see the relevant directory.

## Build Instructions

Building the *Testbed* requires some library dependencies to be available:
the project library, the GLFW version 3 library, and the GLEW version 2 library.
These libraries may be installed from source, as described below, or they can be
installed using a package manager such as `apt-get`, `yum`, NuGet, or `vcpkg`.
On Windows, [`vcpkg`](https://github.com/Microsoft/vcpkg) is recommended, as the build system is equipped to find and link the libraries cleanly (see below).

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
The Testbed GUI application is currently building for Windows and Linux using CMake
and for Mac OS X using Xcode.

#### Linux Using CMake

Assuming that GLFW and GLEW are installed under `/usr/local`:

    cd ${BUILD_DIR} && cmake -DPLAYRHO_BUILD_TESTBED=ON ${CMAKEFILE} && make

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

Windows is less intelligent about locating GLFW and GLEW. There are at least two
ways to handle this issue: use `vcpkg` or specify the library location using CMake variables.

##### Using `vcpkg`

 1. Install [`vcpkg`](https://github.com/Microsoft/vcpkg) if necessary.
 2. Run `vcpkg integrate install`, and copy the command line option starting from `-DCMAKE_TOOLCHAIN_FILE=[...]` to a notepad file or elsewhere.
 3. Install the libraries using:
     `vcpkg install glew:x64-windows-static`
     `vcpkg install glfw3:x64-windows-static`
  or use `x86` if running on a 32-bit platform.
 4. Create a build directory somewhere on your PC if you haven't already, and navigate to it within a command prompt.
 5. Run `cmake -DCMAKE_TOOLCHAIN_FILE=[...] -DVCPKG_TARGET_TRIPLET=x64-windows-static -DPLAYRHO_BUILD_TESTBED=ON [source code directory]`, inserting the command line option copied from step 2, and specifying a relative or absolute path to the source code directory. You may use any additional command line options as described in the [CMake documentation](https://cmake.org/cmake/help/v3.9/manual/cmake.1.html) to further customize your build.
 6. Build the generated project.

#### Other installations

If you have installed from source, or used a package manager other than `vcpkg` on Windows, you will likely need to explicitly specify library locations using CMake variables. While this method was tested on Windows 10, a similar procedure should work on Linux and OS X. The CMake variables may be set using the `-D` command line option, using a CMake cache file, or from within the CMake-gui. The variables which must be specified are:

 - `GLEW_INCLUDE_DIR` : the path to the GLEW include directory, ending with `\include`
 - `GLEW_LIBRARY` : the full filepath to the `glew.lib` file
 - `GLFW_INCLUDE_DIRS` : the path to the GLFW include directory, ending with `\include`
 - `GLFW_LIBRARY_DIRS` : the path to the folder containing the `glfw.lib` file
 - `OPENGL_INCLUDE_DIR` : the path to the OpenGL include directory, ending with `\include`
 - `GLFW_STATIC_LIBRARIES` : `glfw.lib`

If the libraries were installed using the NuGet package manager, a CMake cache file setting these variables might resemble the following:

    set(GLEW_INCLUDE_DIR "$ENV{NUGET_PATH}/glew.1.9.0.1/build/native/include" CACHE PATH "glew header directory")
    set(GLEW_LIBRARY "$ENV{NUGET_PATH}/glew.1.9.0.1/build/native/lib/v110/x64/Release/static/glew.lib" CACHE FILEPATH "glew library")
    set(GLFW_INCLUDE_DIRS "$ENV{NUGET_PATH}/glfw.3.2.1/build/native/include" CACHE PATH "glfw header directory")
    set(GLFW_LIBRARY_DIRS "$ENV{NUGET_PATH}/glfw.3.2.1/build/native/lib/v140/x64/static" CACHE PATH "glfw directory")
    set(OPENGL_INCLUDE_DIR "$ENV{NUGET_PATH}/glfw.3.2.1/build/native/include" CACHE PATH "opengl header directory")
    set(GLFW_STATIC_LIBRARIES "glfw3.lib" CACHE FILEPATH "glfw library")
    set(PLAYRHO_BUILD_TESTBED ON CACHE BOOL "Build PlayRho Testbed GUI application")

where the environment variable `NUGET_PATH` has been set to the NuGet root folder. The project may be built using the following steps, assuming we start in a directory containing `MyCacheFile.cmake`:
	
    mkdir PlayRhoBuild
    cd PlayRhoBuild
    cmake ../MyCacheFile.cmake [source code directory]

You may use any additional command line options as described in the [CMake documentation](https://cmake.org/cmake/help/v3.9/manual/cmake.1.html) to further customize your build.

## Usage Instructions

To run the demos under MS Visual Studio, set `Testbed` as your startup project and press <kbd>F5</kbd> to debug (can step through the code) or <kbd>Ctrl</kbd>+<kbd>F5</kbd> to run without debugging (runs faster).

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
