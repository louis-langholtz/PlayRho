# Building & Installation Instructions

This project can be built a few different ways depending on the target platform and what exactly is desired to be built. It can be installed then after being built.

The following sections provide per-platform and per-tool build and install instructions specific to simply building the PlayRho library. The library can then be included and linked into your application(s).

Additionally, PlayRho provides some demo/test applications which may provide extra instructions for building those after getting the library built:
- For the `Testbed` GUI application, see the [`Testbed/` folder's](Testbed/)
  `README.md` file.

**NOTE**: These sources make heavy use of the standard C++ library [`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to help confirm correctness of the code logic. As such, this library *must* be compiled with the `NDEBUG` pre-processor macro enabled, and of course optimization turned on, to see any kind of performance.

## Prerequisite Tasks

1. Make sure you have the tools that will be needed.
  - **CMake**: If you don't have version 3.16 or higher of CMake installed on your system, the latest releases of CMake can be obtained from https://cmake.org/.
  - **A C++17 compiler suite**: That could be Microsoft's Visual C++, GNU's GCC, or LLVM's Clang. On Microsoft Windows platforms, you can get a complete C++17 compiler suite by getting Microsoft's Visual Studio 2017 from https://www.visualstudio.com/downloads/.
  - **git**: Needed only if you want to do development work. If you don't already have Git installed but you want to install it, you can get it from: https://git-scm.com/downloads.
2. Decide where you want to put the source code.
3. Download the PlayRho source code to your chosen location.

Here's instructions for some ways to download the source code to your computer.

### Download Sources As A ZIP File From Your Browser

This is an option if you have software for reading and expanding ZIP files.

1. Click on the `Clone or download` button from https://github.com/louis-langholtz/PlayRho;
2. Click on the `Download ZIP` link.
3. Decide where you want to place the source code and then use your ZIP reading and expanding software to put it there for you.

### Open the Sources in Your Desktop From Your Browser

This is an option if you have (or want to have) the GitHub Desktop Application installed on your system and you want to use it.

### Download Sources Via A Command Line Prompt & Git Clone

From a command prompt like a Bash Shell, enter code such as the following where `${YOUR_CHOSEN_PATH}` is the directory/folder you decided to hold the sources:

    cd ${YOUR_CHOSEN_PATH}
    git clone --recursive https://github.com/louis-langholtz/PlayRho.git

This should download the sources into the `${YOUR_CHOSEN_PATH}` directory under the sub-directory of `PlayRho`.

## Building From A Bash Shell Using CMake and Make

This folder contains a CMake configuration file `CMakeLists.txt`. It can be used by `cmake` to build the library and all of the application targets on Linux platforms with either GCC C++ or Clang C++. This is the currently suggested way to build things for Linux. Simply run `cmake . && make`. To install the built targets, then run `make install`. This may require a tool like `sudo` to escalate your privileges in order to be able to install into the established folders. The CMake configuration file has a few options that can be modified to build different targets.

Note: if you have trouble building with these instructions, you can also see how PlayRho gets built on the [Travis](https://travis-ci.org) Continuous Integration (CI) system, by looking at the [`.travis.yml`](.travis.yml) file's `script` definition.

## Building On Mac OS X Using Xcode

The [`Build/xcode5`](Build/xcode5) folder contains project files for Xcode. The Xcode project file can be used to build the library and all of the application targets on and for the Mac OS X platform.

## Building On Windows Using CMake-GUI and Visual Studio

Start CMake up with the location of the source code identified for it. By default, only `PLAYRHO_BUILD_STATIC` has a defined value and that should be all that's necessary for simply building the library itself.
Other `PLAYRHO_BUILD_*` CMake variables can be used to build additional things like the `UnitTest` or `Testbed` applications.
Press *Configure* or *Generate*.
If you haven't run CMake before for this project, CMake won't already have a cache of settings,
and will ask you some additional information like which generator to use.
Select Visual Studio 15 2017 or newer to generate the project and solution files for that version of Visual Studio and to invoke that version whenever you press the *Open Project* button.
If you want 64-bit support, you'll need to choose the optional platform generator for your system - that would be `x64` to build for 64-bit Intel X64 platforms.
Use the default native compilers.
Then press the *Finish* button.
CMake may create new variables or need your help especially if you want to also build an optional application like the `Testbed`.
If you change any CMake variables, be sure to press the Generate button.
Finally, press the *Open Project* button. This will open up Visual Studio.

From within Visual Studio, select "Release" and initiate a build.

Note: if you have trouble building with these instructions, you can also see how PlayRho gets built on the [AppVeyor](https://ci.appveyor.com/) Continuous Integration (CI) system, by looking at the [`appveyor.yml`](appveyor.yml) file's `build_script` definition.

For the `Testbed` GUI application, see the [`Testbed/` folder's](Testbed/)  `README.md` file.

### Deprecated: Using Project Files

The [`Build/vs2017`](Build/vs2017) folder contains project files for Visual Studio 2017. These project files may be usable to build the library on and for the Windows platform. Note that these are not intended to be used as the primary way to build the project.

## Older Build Instructions

### CMake

PlayRho uses CMake to describe the build in a platform independent manner. First download and install cmake from https://cmake.org/.

For Microsoft Visual Studio:
- Run the cmake-gui
- Set the source directory to the path of PlayRho on your PC (the folder that contains this file).
- Set the build directory to be the path of `PlayRho/Build` on your PC.
- Press the *Configure* button and select your version of Visual Studio.
- You may have to press the *Configure* button again.
- Press the *Generate* button.
- Open `PlayRho/Build/PlayRho.sln`.
- Set the `Testbed` or `HelloWorld` as your startup project.
- Press F5 or Ctrl-F5 to run.

For Unix platforms, enter the following on a terminal (replacing `$PLAYRHOPATH` with the directory
where this file is located):

    cd $PLAYRHOPATH/Build
    cmake -DPLAYRHO_INSTALL=ON -DPLAYRHO_BUILD_SHARED=ON ..
    make
    make install

You might want to add `-DCMAKE_INSTALL_PREFIX=/opt/PlayRho` or similar to the cmake call to change
the installation location. make install might need sudo.
