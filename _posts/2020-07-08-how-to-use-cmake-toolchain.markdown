---
layout: post
title:  "Comprehensive CMake Tutorial"
date:   2020-07-8 14:58:00
categories: cmake
brief: "A CMake tutorial by example"
---

# Introduction

CMake is an open source C++ build system, which supports almost all the platform. It makes building cross-platform C++ project much easier. It has its own syntax. In this tutorial I will cover all the feature I know.

# Basis
Build a CMake project needs a file named CMakeLists.txt. It often places in the root of the project directory. like:
```
/
  include/
    ..
  src/
    ..
  CMakeLists.txt
```

Here is the simple form of CMakeLists.txt
```
cmake_minimum_required(VERSION 3.13)
project(Demo)
add_executable(DemoProgram src/main.cpp)
```
The first line declare the minimum CMake version it required.
The second line declare the project name(in Visual Studio means solution name). The 3rd line defined the executable target call DemoProgram which compiled with src/main.cpp.

Now we have our CMakelists.txt. We build the project with:
```
mkdir build
cmake ..
cmake --build . --config Debug
```
This is a common way of building a project in its source code.
At first, we create a dir called build under the root dir. Next, We go into the new dir and generate build scripts(In linux it's the Makefile, In windows it's a VS Solution). Last We use cmake tool to build the project with debug configuration.

If you are under linux, you can also call
```
make
```
to build the project.

If you are in windows, you can also open the .sln with MS Visual Studio.


# More commands
## target_include_directory
This command add the include dirs to the target. For example:
```
target_include_directory(DemoProg include/)
```
Above line add the directory "include/" into the target DemoProg's include dirs.

## set
the set command will set the environment variable. For example:
```
set(BOOST_INSTALL_PATH ${SOLUTION_DIR}/3rd-part/boost)
```
The above code alias the path "${SOLUTION_DIR}/3rd-part/boost" to variable BOOST_INSTALL_PATH. after that ${BOOST_INSTALL_PATH} will
become the path you set before.

## target_link_libraries
Just like target_include_directory. The command add the library into target's libraries. The syntax is liked following:
```
target_link_libraries(TargetName a.lib b.lib...)
```
or if you have both debug and release version of a lib
```
target_link_libraries(TargetName debug a-debug.lib optimized a-release.lib)
```

To be continued...