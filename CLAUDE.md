# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

**Tessellator** is a C++17 mesher designed to generate meshes and data structures optimized for FDTD algorithms. The project supports multiple meshing strategies (staircased, conformal, offgrid) and includes extensive test coverage using Google Test.

## Key Architecture

### Core Components

The codebase is organized into modular namespaces under `src/`:

- **`meshers/`** – Abstract mesh generation interface with three implementations:
  - `StaircaseMesher` – Generates staircased meshes from geometric inputs
  - `ConformalMesher` – Creates conformal meshes with fixed-distance grid plane intersections
  - `OffgridMesher` – Produces offgrid meshes (graded rectilinear support)
  - All inherit from `MesherBase` which handles input mesh processing and grid construction

- **`core/`** – Fundamental mesh processing algorithms:
  - `Slicer` – Slices input geometry along grid planes
  - `Staircaser` – Converts sliced geometry into staircase patterns
  - `Smoother` – Post-processing to smooth mesh irregularities
  - `Collapser` – Merges adjacent elements for optimization
  - `Snapper` – Aligns geometry with grid coordinates

- **`types/`** – Core data structures:
  - `Mesh` – Central data structure holding coordinates, elements (nodes/lines/surfaces/volumes), and metadata
  - `Mesher` – Abstract interface that all meshers implement
  - `Vector` – Template-based 3D vector type
  - `Grid` – Rectilinear grid representation (3D array of coordinate positions)

- **`cgal/`** – Optional CGAL-based advanced geometry operations (controlled by `TESSELLATOR_ENABLE_CGAL`):
  - `Filler` – Fills closed polyhedrons using CGAL
  - `Delaunator` – Triangulation operations
  - `Manifolder` – Manifold repair utilities
  - `Repairer` – Polyhedron repair tools
  - `HPolygonSet`, `PolyhedronTools` – Polygon/polyhedron manipulation

- **`utils/`** – Utility functions:
  - `MeshTools`, `GridTools` – Mesh/grid manipulation
  - `Geometry` – Geometric calculations
  - `ConvexHull` – Convex hull computation
  - `CoordGraph`, `ElemGraph` – Topological graph representations
  - `RedundancyCleaner` – Removes redundant geometric data

- **`app/`** – Application interface:
  - `launcher` – CLI entry point and argument parsing
  - `vtkIO` – VTK file import/export (STL, VTK formats)

### Data Flow

Input geometry (STL/VTK) → VTK I/O → Mesher (chooses algorithm) → Core processing (slicer/staircaser/smoother) → Grid alignment → Output mesh (VTK)

## Build System

Uses **CMake 3.20+** with presets and vcpkg for dependency management.

### Build Presets

```bash
# GNU/Linux (Ninja)
cmake --preset gnu -S . -B build
cmake --build build -j

# Windows (MSBuild)
cmake --preset msbuild -S . -B build
cmake --build build --config Release -j
```

### CMake Options

- `TESSELLATOR_ENABLE_TESTS` (ON by default) – Build test suite
- `TESSELLATOR_ENABLE_CGAL` (ON by default) – Enable CGAL-based geometry operations
- `TESSELLATOR_EXECUTION_POLICIES` (OFF by default) – Parallel execution policies

### Dependencies

Managed via vcpkg manifest:
- **Required**: VTK, Boost
- **Optional**: CGAL (if `TESSELLATOR_ENABLE_CGAL=ON`)

To set up locally, create a `CMakeUserPreset.json` file:

```json
{
  "version": 4,
  "include": ["CMakePresets.json"],
  "configurePresets": [
    {
      "name": "gnu-local",
      "environment": {
        "VCPKG_ROOT": "~/workspace/vcpkg/"
      },
      "cacheVariables": {
        "TESSELLATOR_ENABLE_CGAL": true
      },
      "inherits": "gnu"
    }
  ]
}
```

## Common Development Tasks

### Build

```bash
cmake --preset gnu -S . -B build
cmake --build build -j
```

Output binaries go to `build/bin/` and libraries to `build/lib/`.

### Run Tests

```bash
# Run all tests
build/bin/tessellator_tests

# Run specific test suite (e.g., all StaircaseMesher tests)
build/bin/tessellator_tests --gtest_filter=StaircaseMesher*

# Run single test method
build/bin/tessellator_tests --gtest_filter=StaircaseMesherTest.TestName
```

Tests are organized by module under `test/` mirroring the source structure.

### Run Application

```bash
# Build and run with a tessellator JSON input file
build/bin/tessellator -i mesh_definition.tessellator.json
```

## Code Style & Patterns

- **C++17** standard; no C++20+ features
- **Namespaces**: Organized as `meshlib`, `meshlib::meshers`, `meshlib::core`, etc.
- **Mesh representation**: All geometry flows through the `Mesh` object with its `Element` struct (type: Node/Line/Surface/Volume)
- **Grid**: Stored as `Grid = std::array<std::vector<double>, 3>` (coordinate arrays per axis)
- **Logging**: Use static `MesherBase::log()` methods with indentation level for hierarchical output
- **Vertex references**: Elements reference coordinates by index (`CoordinateId`)

## Testing

Google Test (GTest) framework with fixtures. Tests for each module live in `test/<module>/`. Key fixtures:

- `MeshFixtures.h` – Shared mesh construction utilities for tests

**Important**: When modifying core algorithms (Slicer, Staircaser, Smoother, Snapper, Collapser), update corresponding tests in `test/core/` and `test/meshers/`.

## JSON Input Format

The tessellator expects a JSON file with two main entries:

### Grid Definition

Define the grid structure using either uniform or rectilinear (graded) cells:

```json
"grid": {
  "numberOfCells": [20, 20, 30],
  "boundingBox": [[-1, -1, -1], [1, 1, 2]]
}
```

Or for non-uniform grids:

```json
"grid": {
  "planes": [
    [600, 603.25],
    [25.0, 30.5, 92, 130, 1000],
    [1000, 1111, 1111.1]
  ]
}
```

### Object Definition

```json
"object": {"filename": "geometry.stl"}
```

The filename is relative to the JSON file location.

## File Organization

- **Headers** (`.h`) – Include guards, template implementations
- **Implementations** (`.cpp`) – Separate compilation units
- **Test files** – Follow `<ComponentName>Test.cpp` pattern with GTest syntax

## Known Constraints

- CGAL-dependent code is conditionally compiled; check `if (TESSELLATOR_ENABLE_CGAL)` guards when making changes
- Grid must be rectilinear; arbitrary unstructured grids are not supported
- Mesh must be manifold for some operations (see `Manifolder`)
- Elements reference coordinates by index, not direct storage
