# Building & Installation

This project can be built different ways depending on things like what components are desired to be built and the target platform.
It can then be installed into your system, or included and linked into your applications.

## Available Components

Here's the list of available components and the command line arguments they each minimally need to be a part of the build:
- [Library](PlayRho/) (required).
- [Hello world application](HelloWorld/) (optional): add the `-DPLAYRHO_BUILD_HELLOWORLD=ON` argument to the configuration.
- [Unit test application](UnitTests/) (optional): add the `-DPLAYRHO_BUILD_UNIT_TESTS=ON` argument to the configuration.
- [Benchmark application](Benchmark/) (optional): add the `-DPLAYRHO_BUILD_BENCHMARK=ON` argument to the configuration.
- [Testbed application](Testbed/) (optional): add the `-DPLAYRHO_BUILD_TESTBED=ON` argument to the configuration.

Some components may need additional arguments that may also depend on the target platform.
See the documentation for each component to learn more about that component and about its specific prerequisites and necessary command line arguments.

## Supported Platforms

All components are known to build, and the library to validate, on the following target platforms (as confirmed through this project's continuous integration setup):
- [macOS 11 (Release & Debug)](.github/workflows/macos.yml).
- [Ubuntu Linux (Release)](.github/workflows/linux.yml).
- [Ubuntu Linux (Debug)](.github/workflows/linux-debug.yml).
- [Windows "2019" (Win32, x64 for Release & Debug)](.github/workflows/windows.yml).

While the library itself should be buildable on any platform having a C++17 or higher standards compliant compiler,
other components may have dependencies that make them harder to build and available to less target platforms.

**NOTE**: These sources make heavy use of the standard C++ library [`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to help confirm correctness of the code logic. As such, this library needs to be compiled with the `NDEBUG` pre-processor macro enabled to disable these asserts, and of course optimization turned on, to see any kind of performance. Add the command line arguments `--config Release` to the build step to ensure built code is optimized and the performance degrading asserts are not included.

## Prerequisites

The necessary installed prerequisites depend on the components to be built and the platform they are built on.
Follow the above links for each component you want to build to read about the prerequisites needed for that component
and to ensure that they are installed.

All of the components minimally require the following:
- A command line shell/window like `bash`. For Windows, see https://learn.microsoft.com/en-us/windows/terminal/.
- **git**: If you don't already have Git installed, it can be obtained from: https://git-scm.com/downloads.
- **A C++17 compiler suite**: That could be Microsoft's Visual C++, GNU's GCC, or LLVM's Clang. On Microsoft Windows platforms, you can get a complete C++17 compiler suite by getting Microsoft's Visual Studio from https://www.visualstudio.com/downloads/.
- **CMake**: If you don't have version 3.16.3 or higher of CMake installed on your system, the latest releases of CMake can be obtained from https://cmake.org/.

## Steps

All of these steps should be run from the same working directory.

### Download The Repository

If you haven't already downloaded the repository, run:
```sh
git clone --recurse-submodules https://github.com/louis-langholtz/PlayRho.git
```

### Configure

This step sets up things for future steps, like what components are to be built, how they are to be built, and where they are to be installed.
This step depends heavily on the components chosen, how you want them built, and possibly the platform they are built on.
Follow the above links for each component you're interested in to read about the configuration arguments needed for that component and possibly for the target platform.

For example, to only build the library component using the CMake default generator and compiler environment, run the following:

```sh
cmake -S PlayRho -B PlayRhoBuild
```

Alternatively, add the configuration arguments needed for each desired component and your platform.
Then run the following where `$ConfigOptions` is the accumulated configuration options for the components you want:

```sh
cmake -S PlayRho -B PlayRhoBuild $ConfigOptions
```

#### Additional Options

For a listing of non-advanced configuration options, run:
```sh
cmake -LH -S PlayRho -B PlayRhoBuild
```

For example:
- To build everything to use double precision floating point arithmetic (instead of single precision), add `-DPLAYRHO_REAL_TYPE=double`.
- To build everything using shared linkage (instead of static linkage), add `-DBUILD_SHARED_LIBS=ON`.
- To set the installation prefix, add an argument like `-DCMAKE_INSTALL_PREFIX=/opt/PlayRho`.

### Build

The following compiles the code using the generator and compiler environment established on configuration, and also specifies an optimized "Release" build:

```sh
cmake --build PlayRhoBuild --config Release
```

### Install

This step is optional.

To install the configured components into the CMake default system locations, run the following command.
```sh
cmake --install PlayRhoBuild
```

## TLDR

```sh
git clone --recurse-submodules https://github.com/louis-langholtz/PlayRho.git
cmake -S PlayRho -B PlayRhoBuild
cmake --build PlayRhoBuild --config Release
cmake --install PlayRhoBuild
```

These steps assume that:
- You only want to build the library component and want it optimized for fastest performance.
- You want to install these components into CMake's standard system paths.
- The necessary prerequisites for the components have already been installed.

