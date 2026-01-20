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
        "TESSELLATOR_ENABLE_CGAL": true
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
The two main entries are as follows:

### `<grid>`
This object must always be present and contains the structure of the grid, which will be used to slice and adjust the mesh provided. It must contain one of these two sets of entries:

- `<numberOfCells>`: is an array of three positive integers which indicate the number of cells in each Cartesian direction. In case of having this entry, it also must contain a `<boundingBox>`:
  - `<boundingBox>` is represented by an array which contairs two triplets of integers, representing the minimum and maximum values of the gread in each cartesian direction.

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

### `<object>`
This contains the information about the mesh file. It must contain the following entry:

- `filename`: with an string containing the name of the mesh file. Its location is relative to that of the json file.

  Example:

```json
  "object": {"filename": "thinCylinder.stl"}
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
