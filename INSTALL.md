# Building & Installation

This project can be built different ways depending on things like what components are desired to be built and the target platform.
It can then be installed into your system, or included and linked into your applications.

The following instructions are oriented towards building the project yourself using [CMake](https://cmake.org/), and doing so through a command line interface.

Alternatively, if you don't want to build the project yourself, you may be able find a package for your platform through a site like [repology.org](https://repology.org/projects/?search=playrho).

## Supported Platforms

All components are known to build, and the library to validate, on the following target platforms (as confirmed through this project's continuous integration setup):
- [macOS 11 (Release & Debug)](.github/workflows/macos.yml).
- [Ubuntu Linux (Release)](.github/workflows/linux.yml).
- [Ubuntu Linux (Debug)](.github/workflows/linux-debug.yml).
- [Windows "2019" (Win32, x64 for Release & Debug)](.github/workflows/windows.yml).

While the library itself should be buildable on any platform having a C++17 or higher standards compliant compiler, other components may have dependencies that make them harder to build and available to less target platforms.

**NOTE**: These sources make heavy use of the standard C++ library [`assert`](http://en.cppreference.com/w/cpp/error/assert) macro to help confirm correctness of the code logic.
As such, this library needs to be compiled with the `NDEBUG` pre-processor macro enabled to disable these asserts, and of course optimization turned on, to see any kind of performance.
Add the command line arguments `--config Release` to the build step to ensure built code is optimized and the performance degrading asserts are not included.

## Prerequisites

The necessary installed prerequisites depend on the components to be built and the platform they are built on.
All of the components minimally require the following:

- **CMake**: If you don't have version 3.16.3 or higher of CMake installed on your system, the latest releases of CMake can be obtained from https://cmake.org/. Note that it's possible to skip this prerequisite, but it's not recommended nor as well supported.
- **A C++17 compiler suite**: That could be Microsoft's Visual C++, GNU's GCC, or LLVM's Clang. On Microsoft Windows platforms, you can get a complete C++17 compiler suite by getting Microsoft's Visual Studio from https://www.visualstudio.com/downloads/.
- A command line shell/window like `bash`. For Windows, see https://learn.microsoft.com/en-us/windows/terminal/.
- **git**: If you don't already have Git installed, it can be obtained from: https://git-scm.com/downloads.

## Steps

All of these steps assume they're run from the same working directory.

### Get Source Code Access

If you don't already have access to the source code, use **one** of the following methods to get it:

<details>
<summary>Download the source code as a compressed file.</summary>
If you only want to build the source code and aren't interested in its development, you can just download a *ZIP*  or *tar* file of the code.
For example, to download the latest release source code:

1. Visit [latest release](https://github.com/louis-langholtz/PlayRho/releases/latest) and follow the link for the source code asset of the type you want to download.
1. Uncompress and extract it (i.e. unzip or untar it), if not done automatically for you by your download application.
1. Rename the extracted directory to `PlayRho` (or whatever but then remember the name and replace `PlayRho` in these next steps with it).
</details>

<details>
<summary>Git clone the source code.</summary>
If you might be interested in helping development of this project or prefer using `git`:

```sh
git clone https://github.com/louis-langholtz/PlayRho.git
```

This should work fine for building the base library component.
This should also work fine now for the application components, so long as you use CMake in the following steps.
</details>

<details>
<summary>Git recursive clone the source code.</summary>
Alternatively, like if you want to try building without CMake via Xcode for example, do a recursive clone:

```sh
git clone --recurse-submodules https://github.com/louis-langholtz/PlayRho.git
```
</details>

### Configure

This step configures things for future steps, like what components are to be built, how they are to be built, and where they are to be installed.
This step depends heavily on the components chosen, how you want them built, and possibly the platform they are built on.

For example, to build just the [library](Library) component using the CMake default generator and compiler environment, run the following:

```sh
cmake -S PlayRho -B PlayRhoBuild
```

#### Configuration Options

Append the arguments for any additional components you want to build.
Here's the list of available components and the arguments they each **minimally** need to be a part of the build:

- [Hello world](HelloWorld/) application, append: `-DPLAYRHO_BUILD_HELLOWORLD=ON`.
- [Unit test](UnitTests/) application, append: `-DPLAYRHO_BUILD_UNIT_TESTS=ON`.
- [Benchmark](Benchmark/) application, append: `-DPLAYRHO_BUILD_BENCHMARK=ON`.
- [Testbed](Testbed/) application, append: `-DPLAYRHO_BUILD_TESTBED=ON`.
- [Documentation](Documentation/) text/html, append: `-DPLAYRHO_BUILD_DOC=ON`.

<strong>Some of these may need additional arguments that may also depend on the target platform.</strong>
Follow the links for each component you're interested in to see specific documentation for that component such as prerequisites it may have and additional command line arguments.

Here's some **component independent** options and the arguments they need:

- Export compile commands (for tools like `run-clang-tidy`), append: `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.
- Use double precision floating point (instead of single precision), append: `-DPLAYRHO_REAL_TYPE=double`.
- Do shared linkage (instead of static linkage), append: `-DBUILD_SHARED_LIBS=ON`.
- Override default compiler tool chain like from `/opt/homebrew/opt/llvm/`, add **before** the `cmake` command: `CC=/opt/homebrew/opt/llvm/bin/clang CXX=/opt/homebrew/opt/llvm/bin/clang++ LDFLAGS='-L/opt/homebrew/opt/llvm/lib/c++'`.

For a listing of non-advanced CMake configuration options, run:

```sh
cmake -LH -S PlayRho -B PlayRhoBuild
```

### Build

The following compiles the code using the generator and compiler environment established on configuration, and also specifies an optimized "Release" build:

```sh
cmake --build PlayRhoBuild --config Release
```

### Install

This step is optional.

To install the configured components into CMake's default locations, just run the following command:

```sh
cmake --install PlayRhoBuild
```

To specify a prefix like `PlayRhoInstall`, append the following arguments: `--prefix PlayRhoInstall`.

See CMake's [Install a Project](https://cmake.org/cmake/help/latest/manual/cmake.1.html#install-a-project) webpage for more info.

## TL;DR

```sh
git clone https://github.com/louis-langholtz/PlayRho.git
cmake -S PlayRho -B PlayRhoBuild
cmake --build PlayRhoBuild --config Release
cmake --install PlayRhoBuild
```

These steps assume that:
- You only want to build the library component and want it optimized for fastest performance.
- You want to install it into CMake's default paths.
- The necessary prerequisites have already been installed.

