# DetourWrapper

DetourWrapper is a C++/CLI project that provides a managed C++ wrapper for the C++ Detour library. This wrapper allows you to use Detour functions in your server project, which is written in C#. By bridging the gap between C++ and C#, you can achieve seamless code integration for your server application.

## Table of Contents

- [Introduction]
- [Features]
- [Getting Started]
- [Contributing]
- [License]

## Introduction

Detour is a powerful library for hooking and redirecting function calls in native C++ applications. However, when working on managed code projects like the EQOA server written in C#, interfacing with native C++ libraries can be challenging. DetourWrapper aims to simplify this process by providing a managed C++ interface that can be easily consumed in your C# project.

## Features

- **Managed C++ Wrapper:** DetourWrapper offers a managed C++/CLI interface that makes it accessible from C#.

- **Easy Integration:** You can seamlessly integrate Detour functions into your EQOA server project without dealing with complex interop code.

## Getting Started

To use DetourWrapper in your EQOA server project, follow these steps:

1. Clone this repository to your local machine:
   https://github.com/bsekinger/DetourWrapper.git
2. This project uses OpenGL Mathematics (GLM). Clone it to your local machine:
	https://github.com/g-truc/glm.git
3. Clone the example application which provides sample code snippets demonstrating how to use DetourWrapper in your EQOA server application:
	https://github.com/bsekinger/DetourWrapperTest.git

## contributing
std::rnd

## License
DetourWrapper is licensed under the MIT License.