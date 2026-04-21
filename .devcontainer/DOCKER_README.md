# Dev Container Setup

This directory contains the configuration for a Docker-based development environment for Tessellator.

## Quick Start

### Prerequisites
- Docker installed and running
- VS Code with the "Dev Containers" extension

### Opening the Project in Dev Container

1. Open this repository in VS Code
2. Click the green icon in the bottom-left corner and select **"Reopen in Container"**
3. VS Code will build the Docker image and start the container automatically

The entire workspace will be mounted at `/workspace` inside the container.

## What's Included

The dev container provides:
- Pre-configured C++ development environment
- CMake and Ninja build tools
- All project dependencies (VTK, Boost, CGAL via vcpkg)
- VS Code extensions:
  - **C/C++ Tools** – Code navigation, IntelliSense, debugging
  - **CMake Tools** – CMake project management
  - **LLDB** – Debugger for C++

## Building and Testing

Once inside the container:

```bash
# Configure the project
cmake --preset docker -S . -B build

# Build
cmake --build build -j

# Run tests
build/bin/tessellator_tests

# Run the application
build/bin/tessellator -i <input.tessellator.json>
```

## Notes

- The container runs as root user
- CMake is pre-configured to skip auto-configure on file open for better performance
- Debugging with LLDB is enabled with appropriate capabilities
