# Tessellator mesher

[![License](https://img.shields.io/badge/License-GPL_3.0-blue.svg)](https://opensource.org/licenses/gpl-3.0)
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

## Contributing

## Citing this work
If you use this software, please give proper attribution by citing it as indicated in the [citation](CITATION.cff) file. 


## Copyright and license
This code and its copyright is property of to the University of Granada (UGR), CIF: Q1818002F, www.ugr.es. UGR has licensed its distribution under terms of the GPL-3.0 license (see [LICENSE](LICENSE) file) with the name of `meshlib` 
