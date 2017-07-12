# Building & Installation Instructions

This project can be built a few different ways depending on the target
platform and what exactly is desired to be built. It can be installed then
after being built.

Specific instructions for building the `Testbed` GUI
application is available in the [`Testbed/` folder's](Testbed/)
`README.md` file.

The following sections provide general per-platform and per-tool build and
install instructions.

## Linux

### Using CMake

This folder contains a CMake configuration file `CMakeLists.txt`. It can be
used by `cmake` to build the library and all of the application targets on
Linux platforms with either GCC C++ or Clang C++. This is the currently
suggested way to build things for Linux. Simply run `cmake . && make`.
To install the built targets, then run `make install`. This may require
a tool like `sudo` to escalate your privileges in order to be able to install
into the established folders. The CMake configuration file has a few options
that can be modified to build different targets.

## Mac OS X

### Using Xcode

The [`Build/xcode5`](Build/xcode5) folder contains project files for Xcode.
The Xcode project file can be used to build the library and all of the
application targets on and for the Mac OS X platform.

### Using CMake

This has not been tested. No further information is currently available.

## Windows

### Using Visual Studio 2017

The [`Build/vs2017`](Build/vs2017) folder contains project files for
Visual Studio 2017. These project files can be used to build the library
on and for the Windows platform.

## Older Build Instructions

### premake

For other platforms you need to run premake in this directory. You can get premake here:
http://industriousone.com/premake

For example, on Linux, you would type:
premake4 gmake

This will create a gmake folder in the Build directory. From there you can run:
make config="debug"

If you have build problems, you can post a question here:
http://box2d.org/forum/viewforum.php?f=7

### cmake

PlayRho uses CMake to describe the build in a platform independent manner.

First download and install cmake from cmake.org

For Microsoft Visual Studio:
- Run the cmake-gui
- Set the source directory to the path of PlayRho on your PC (the folder that contains this file).
- Set the build directory to be the path of PlayRho/Build on your PC.
- Press the Configure button and select your version of Visual Studio.
- You may have to press the Configure button again.
- Press the Generate button.
- Open PlayRho/Build/PlayRho.sln.
- Set the Testbed or HelloWorld as your startup project.
- Press F5 or Ctrl-F5 to run.

For Unix platforms, say the following on a terminal: (Replace $PLAYRHOPATH with the directory where this file is located.)
	cd $PLAYRHOPATH/Build
	cmake -DPLAYRHO_INSTALL=ON -DPLAYRHO_BUILD_SHARED=ON ..
	make
	make install
You might want to add -DCMAKE_INSTALL_PREFIX=/opt/PlayRho or similar to the cmake call to change the installation location. make install might need sudo.
