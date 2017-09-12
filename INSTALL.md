# Building & Installation Instructions

This project can be built a few different ways depending on the target
platform and what exactly is desired to be built. It can be installed then
after being built.

The following sections provide per-platform and per-tool build and
install instructions specific to simply building the PlayRho library.
The library can then be included and linked into your application(s).

Additionally, PlayRho provides some demo/test applications which may provide
extra instructions for building those after getting the library built:
- For the `Testbed` GUI application, see the [`Testbed/` folder's](Testbed/)
  `README.md` file.

## Prerequisite Tasks

1. Make sure you have the tools that will be needed already available.
  - C++14 Compiler (Installed for your platform).
  - CMake (for Windows, OS X, or Linux),
    Visual Studio 2017 (for Windows or OS X), and/or Xcode (for OS X).
  - Git, if you want to do development work.
2. Decide where you want to put the source code.
3. Download the PlayRho source code to your chosen location.

Here's instructions for some ways to download the source code to your computer.

### Download Sources As A ZIP File From Your Browser

This is an option if you have software for reading and expanding ZIP files.

1. Click on the `Clone or download` button from https://github.com/louis-langholtz/PlayRho;
2. Click on the `Download ZIP` link.
3. Decide where you want to place the source code and then use your ZIP reading
   and expanding software to put it there for you.

### Open the Sources in Your Desktop From Your Browser

This is an option if you have (or want to have) the GitHub Desktop Application
installed on your system and you want to use it.

### Download Sources Via A Command Line Prompt & Git Clone

From a command prompt like a Bash Shell,
enter code such as the following where `${YOUR_CHOSEN_PATH}`
is the directory/folder you decided to hold the sources:
    cd ${YOUR_CHOSEN_PATH}
    git clone --recursive https://github.com/louis-langholtz/PlayRho.git

This should download the sources into the `${YOUR_CHOSEN_PATH}` directory
under the sub-directory of `PlayRho`.

## Building On Linux Systems

### Using CMake

This folder contains a CMake configuration file `CMakeLists.txt`. It can be
used by `cmake` to build the library and all of the application targets on
Linux platforms with either GCC C++ or Clang C++. This is the currently
suggested way to build things for Linux. Simply run `cmake . && make`.
To install the built targets, then run `make install`. This may require
a tool like `sudo` to escalate your privileges in order to be able to install
into the established folders. The CMake configuration file has a few options
that can be modified to build different targets.

## Building On Mac OS X

### Using Xcode

The [`Build/xcode5`](Build/xcode5) folder contains project files for Xcode.
The Xcode project file can be used to build the library and all of the
application targets on and for the Mac OS X platform.

### Using CMake

This has not been tested. No further information is currently available.

## Building On Windows

### Using Visual Studio 2017

The [`Build/vs2017`](Build/vs2017) folder contains project files for
Visual Studio 2017. These project files can be used to build the library
on and for the Windows platform.

### Using CMake

Start CMake up with the location of the source code identified for it.
By default, only `PLAYRHO_BUILD_STATIC` has a defined value and that should be
all that's necessary for simply building the library itself.

## Older Build Instructions

### premake

For other platforms you may be able to run premake in this directory.
Premake can be obtained from:
http://industriousone.com/premake

For example, on Linux, you would type:
premake4 gmake

This will create a gmake folder in the Build directory. From there you can run:
make config="debug"

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

For Unix platforms, say the following on a terminal (replacing `$PLAYRHOPATH` with the directory
where this file is located):

    cd $PLAYRHOPATH/Build
    cmake -DPLAYRHO_INSTALL=ON -DPLAYRHO_BUILD_SHARED=ON ..
    make
    make install

You might want to add `-DCMAKE_INSTALL_PREFIX=/opt/PlayRho` or similar to the cmake call to change
the installation location. make install might need sudo.
