# Testbed

The Testbed GUI application provides a GUI interface to a bunch of visually
presented tests. It may be helpful for learning how to use [this fork of the
Box2D library](https://github.com/louis-langholtz/Box2D).

## Build Instructions

Building the *Testbed* requires some library dependencies to be available:
the Box2D library, the GLFW library, and the GLEW library.

### GLFW

[GLFW](http://www.glfw.org) is an Open Source library that the Testbed depends
on.

Here are steps that I've used to build this library:
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
depends on.

Here are steps that I've used to build this library:
1. Get GLEW.
  - From a ZIP file:
    1. Download GLEW 2.0.0 from
       https://sourceforge.net/projects/glew/files/glew/2.0.0/glew-2.0.0.zip.
    2. Extract the sources: `unzip glew-2.0.0.zip`
  - As a git clone: `git clone https://github.com/nigels-com/glew.git glew`
2. Make sure to have the needed build tools.
  - For Debian/Ubuntu/Mint platforms: `$ sudo apt-get install build-essential libxmu-dev libxi-dev libgl-dev libosmesa-dev git`.
  - For RedHat/CentOS/Fedora:  `$ sudo yum install libXmu-devel libXi-devel libGL-devel git`
3. Run `make` and `make install`.

### Testbed
