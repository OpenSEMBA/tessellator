# Tessellator mesher

[![License](https://img.shields.io/badge/License-GPL_3.0-blue.svg)](https://opensource.org/licenses/gpl-3.0) 
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/mit) 

[![Build and test](https://github.com/OpenSEMBA/tessellator/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/OpenSEMBA/tessellator/actions/workflows/build-and-test.yml)

## Features

Tessellator is a mesher focused on generate meshes and data structures which are suitable for FDTD algorithms. It includes the following capabilities:

- Generate staircased meshes from lines, surfaces, and volumes.
- Support for rectilinear (graded) grids.
- Import/Export in STL or VTK formats.
- Conflict resolution between different layers using a predefined hierarchy.
- Generate conformal meshes with fixed distance intersection with grid planes.

## Compilation

When using presets, make sure to define the environment variable `VCPKG_ROOT` to your `vcpkg` installation.
The standard presets build without CGAL; use the `gnu-cgal` preset for the optional CGAL algorithms.
The launcher application is enabled by default (`TESSELLATOR_ENABLE_APP=ON`) and
requires VTK 7.1 or newer. CMake fails during configuration when the launcher is enabled but VTK is unavailable. To build the mesh library and its non-app tests without VTK, configure with `-DTESSELLATOR_ENABLE_APP=OFF`.

For a clean build with the launcher enabled:

```shell
cmake --preset gnu -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
```

`ctest` automatically runs from the source directory, so no `testData` symlinks
are required.

The launcher prints its build version, commit, compiler, and flags on startup.
For source bundles built without Git, generate `git_info.txt` in a checkout and
include it at the source root before packaging:

```shell
./scripts/write_git_info.sh
```

This can be done using a `CMakeUserPreset.json` file, for example:

```json
{
  "version": 4,

  "include": ["CMakePresets.json"],
  "configurePresets": [
    {
      "name": "gnu-local",
      "displayName": "GNU local",
      "environment": {
        "VCPKG_ROOT": "~/workspace/vcpkg/"
      },
      "cacheVariables": {
        "TESSELLATOR_ENABLE_CGAL": false
      },
      "inherits": "gnu"
    }
  ]
}
```

## Usage

The main binary is `tessellator`, which uses a tessellator json format, which will be explained below.

```shell
    tessellator -i MESH_NAME.tessellator.json
```

## JSON Format
The main entries are as follows:

### `<grid>`
This object must always be present and contains the structure of the grid, which will be used to slice and adjust the mesh provided. It must contain one of these two sets of entries:

- `<numberOfCells>`: is an array of three positive integers which indicate the number of cells in each Cartesian direction. In case of having this entry, it also must contain a `<boundingBox>`:
  - `<boundingBox>` is represented by an array which contains two triplets of integers, representing the minimum and maximum values of the grid in each cartesian direction.

```json
  "grid": {
    "numberOfCells": [20, 20, 30],
    "boundingBox": [
        [-1, -1, -1],
        [ 1,  1,  2]
    ]
  }
```

- `planes`: This array contains other three arrays of floating point numbers. Each number must be in sequential order, from lowest to highest, each value representing the position of every plane forming the cells of the grid. This allows the definition of a non-uniform (rectilinear) grid.

```json
  "grid": {
    "planes": [
        [600, 603.25],
        [25.0, 30.5, 92, 130, 1000],
        [1000, 1111, 1111.1]
    ]
  }
```

### `<object>` or `<objects>`
This contains the information about the mesh file(s). You can specify a single object or multiple objects:

**Single object:**
- `filename`: A string containing the name of the mesh file. Its location is relative to that of the json file.

```json
  "object": {"filename": "thinCylinder.stl"}
```

**Multiple objects:**
- `objects`: An array of object definitions. Each object can have:
  - `filename`: (required) The mesh file name, relative to the JSON file location
  - `group`: (optional) Group name for the object (defaults to filename without extension)
  - `ghost`: (optional boolean, default: false) Excludes the object from cross-object decisions while still meshing and exporting it normally
  - `mesher`: (optional) Override the global mesher settings for this specific object

```json
  "objects": [
    {"filename": "object1.stl", "group": "group1", "ghost": true},
    {"filename": "object2.stl", "group": "group2", "mesher": {"type": "conformal"}}
  ]
```

### `<mesher>`
This optional entry configures the meshing algorithm and its options. If not specified, the staircase mesher is used with default options.

**Mesher types:**
- `staircase` (default): Generates staircased meshes from geometric inputs
- `conformal`: Creates conformal meshes with fixed-distance grid plane intersections

**Mesher options:**

For **staircase** mesher:
- `compress`: (boolean, default: false) Enables surface compression to merge adjacent coplanar quads into larger surfaces
- `splitHexahedra`: (boolean, default: false) Splits filled volumes into one conforming hexahedron per occupied grid cell

For **conformal** mesher:
- `edgePoints`: (non-negative integer, default: `0`) Number of evenly spaced
  candidate snap points added along each grid edge.
  These points are placed in the portion of the edge outside the endpoint exclusion regions defined by
  `forbiddenLength`. Set to `0` to add no interior edge points.
- `forbiddenLength`: (number, default: `0.0`) Fraction of each grid edge kept
  clear next to both endpoints when placing or snapping to edge points. It must be between `0.0` and `0.5`, inclusive.
- `staircaseSharedCells`: (boolean, default: true) Selectively staircases cells occupied by this conformal object and another object

**Global options:**
- `exportGrid`: (boolean, default: true) Controls whether to export the grid file

Example with staircase mesher and compression enabled:
```json
  "mesher": {
    "type": "staircase",
    "options": {
      "compress": true,
      "exportGrid": true
    }
  }
```

### `<output>`
This optional entry controls how multi-object results are written:

- `singleFile`: (boolean, default: false) Writes all objects to one
  `{basename}.tessellator.vtk` file instead of separate per-object mesh files.
  Object groups remain identifiable through the `group` and `groupNames` cell
  attributes. Group names must be unique.

```json
  "output": {
    "singleFile": true
  }
```

Example with conformal mesher:
```json
  "mesher": {
    "type": "conformal",
    "options": {
      "edgePoints": 3,
      "forbiddenLength": 0.001
    }
  }
```

### Output Files
The tessellator generates output files with the following naming convention:
- `{group_name}.tessellator.str.vtk` - Staircase meshed object
- `{group_name}.tessellator.cmsh.vtk` - Conformal meshed object
- `{basename}.tessellator.vtk` - Combined multi-object mesh when `output.singleFile` is true
- `{basename}.tessellator.grid.vtk` - Grid file (if `exportGrid` is true)

### Complete Example
```json
{
  "grid": {
    "numberOfCells": [50, 50, 50],
    "boundingBox": [
      [-100.0, -100.0, -100.0],
      [ 100.0,  100.0,  100.0]
    ]
  },
  "objects": [
    {"filename": "sphere.stl"},
    {"filename": "cylinder.stl", "mesher": {"type": "conformal"}}
  ],
  "mesher": {
    "type": "staircase",
    "options": {
      "compress": true,
      "exportGrid": true
    }
  }
}
```

## Contributing

## Citing this work
If you use this software, please give proper attribution by citing it as indicated in the [citation](CITATION.cff) file. 


## Copyright and license
This code and its copyright is property of to the University of Granada (UGR), CIF: Q1818002F, www.ugr.es. UGR has licensed its distribution under terms of the GPL-3.0 and MIT licenses (see [LICENSE](LICENSE) file) with the name of `meshlib` 

testData/cervezas_alhambra logo has been downloaded from https://cults3d.com/es/modelo-3d/arte/celosia-alhambra-logo-cervezas-alhambra where is available with license CC BY-NC-SA.

## Funding

- Spanish Ministry of Science and Innovation (MICIN/AEI) (Grant Number: PID2022-137495OB-C31)
- European Union, HECATE project. (HE-HORIZON-JU-Clean-Aviation-2022-01)
- iSense Project. In-Situ Monitoring of Electromagnetic Interference. (HE-HORIZON-MSCA-2023-DN-01)
