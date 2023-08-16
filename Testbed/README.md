# Testbed GUI Application

The Testbed GUI application provides a GUI interface to a bunch of visually
presented demos. These may be helpful for testing and/or for learning how to use
[PlayRho](https://github.com/louis-langholtz/PlayRho).

Here in this directory, are three sub-directories:

1. [`Data`](Data/): For data for the application.
2. [`Framework`](Framework/): For code providing a framework for the
   application like the [Test](Framework/Test.hpp) base class.
3. [`Tests`](Tests/): Containing code for demos. These demos all subclass the
   `Test` base class. Take a look herein to see this code and to see images
   of these demos in action. This folder is also where you'd add your own code
   if you wanted it to also run under the Testbed GUI application.

For more specifics, see the relevant directory.

## Prerequisites

This component uses the `Testbed/Framework/imgui` git sub module and has
prerequisites - like the [glfw](https://www.glfw.org) library and sometimes
[glew](https://glew.sourceforge.net) library - depending on the target platform.

These prerequisites can be satisfied as follows as confirmed in continuous integration.
They can probably be satisfied in other ways as well such as by downloading,
configuring, building, and compiling and linking with the sources to these libraries.

### Ubuntu Linux

Or probably any linux using the `apt` package manager.
This platform needs the `glfw` and `glew` libraries in addition to their dependencies.

```sh
sudo apt update
sudo apt install xorg-dev libx11-dev libxmu-dev libxi-dev libgl1 libgl1-mesa-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev libosmesa-dev
sudo apt install libglfw3-dev libglew2.2 libglew-dev
```

### macOS

These steps use the Homebrew package manager for macOS.
This package manager can be obtained from https://brew.sh.
It will need to be installed if it's not already.

This platform needs the `glfw` library in addition to its dependencies; but not the `glew` library.

```sh
brew update
brew install pkg-config glfw
```

### Windows

Building this component for Windows requires the `vcpkg` package manager.
This package manager can be downloaded from https://vcpkg.io.
It will need to be installed if it's not already.
You will also need to note its tool chain file path and use that in the configuration.

This platform needs the `glfw` and `glew` libraries in addition to their dependencies.

For `x64` builds:

```sh
vcpkg install --triplet=x64-windows-static glew glfw3
```

For `Win32` builds:

```sh
vcpkg install --triplet=x86-windows-static glew glfw3
```

## Configuration

This component needs the `PLAYRHO_BUILD_TESTBED` CMake option turned on.
This can be achieved through the command line interface by adding the
`-DPLAYRHO_BUILD_TESTBED=ON` argument to the CMake configuration step's list of arguments.

Some target platforms also need additional arguments.

### Windows "x64"

This platform also needs these arguments:

- `-DCMAKE_TOOLCHAIN_FILE=<your_vcpkg_toolchain_file_path>`.
- `-DVCPKG_TARGET_TRIPLET="x64-windows-static"`.
- `-DCMAKE_GENERATOR_PLATFORM=x64`.

Or altogether as:

```sh
-DPLAYRHO_BUILD_TESTBED=ON -DCMAKE_TOOLCHAIN_FILE=<your_vcpkg_toolchain_file_path> -DVCPKG_TARGET_TRIPLET="x64-windows-static" -DCMAKE_GENERATOR_PLATFORM=x64
```

#### Windows "Win32"

This platform also needs these arguments:

- `-DCMAKE_TOOLCHAIN_FILE=<your_vcpkg_toolchain_file_path>`.
- `-DVCPKG_TARGET_TRIPLET="x86-windows-static"`.
- `-DCMAKE_GENERATOR_PLATFORM=WIN32`.

Or altogether as:

```sh
-DPLAYRHO_BUILD_TESTBED=ON -DCMAKE_TOOLCHAIN_FILE=<your_vcpkg_toolchain_file_path> -DVCPKG_TARGET_TRIPLET="x86-windows-static" -DCMAKE_GENERATOR_PLATFORM=WIN32
```

## Building & Installation

See the project's [documented build and installation steps](../INSTALL.md) and be sure to add the above mentioned command line arguments.

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
