# Building & Installation Instructions

This project can be built a few different ways depending on the target platform and what exactly is desired to be built. It can be installed then after being built.

The following sections provide per-platform and per-tool build and install instructions specific to simply building the PlayRho library. The library can then be included and linked into your application(s).

Additionally, PlayRho provides some demo/test applications which may provide extra instructions for building those after getting the library built:
- For the `Testbed` GUI application, see the [`Testbed/` folder's](Testbed/)
  `README.md` file.

**NOTE**: These sources make heavy use of the standard C++ library [`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to help confirm correctness of the code logic. As such, this library *must* be compiled with the `NDEBUG` pre-processor macro enabled, and of course optimization turned on, to see any kind of performance.

## Prerequisite Tasks

1. Make sure you have the tools that will be needed.
  - **CMake**: If you don't already have CMake installed on your system, it can be obtained from https://cmake.org/.
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

## Building

Use `cmake` to build the library and all of the application targets with a C++17-compatible compiler (GCC C++, Clang C++, Visual Studio will suffice). This is the currently suggested way to build things.

Simply run

  cmake -B build -H . && cmake --build build

To install the built targets (on Unix systems), run `make install`.

This may require admin user permissions (e.g. `sudo`) to be able to install into the established folders.

The CMake configuration file has a few options that can be modified to build different targets.

Note: if you have trouble building with these instructions, you can also see how PlayRho gets built on CI, by checking the [`.travis.yml`](.travis.yml) definition of `script` or [`appveyor.yml`](appveyor.yml) definition of `build_script`.

For the `Testbed` GUI application, see the [`Testbed/` folder's](Testbed/)  `README.md` file.
