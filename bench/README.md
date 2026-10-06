# Tessellator benchmarks

Standalone, dependency-free benchmarks for `meshlib::meshers::StaircaseMesher`.
The sphere case is loaded from `testData/cases/sphere/sphere.stl`; grid sizes are
parameterized per axis.

## Build

Benchmarks are disabled by default and require the app (the sphere STL is loaded
through `vtkIO`):

```bash
cmake --preset gnu -DTESSELLATOR_ENABLE_BENCHMARKS=ON
cmake --build build-rls -j --target tessellator_benchmarks
```

(`build` is a symlink to `build-rls`, and `cmake --build build ...` also works.)

## Run

```bash
build/bin/tessellator_benchmarks [options]
```

| Option | Description |
| --- | --- |
| `--case <surface\|volume\|all>` | Cases to run (default: `all`) |
| `--cells <n[,n,...]>` | Grid cells per axis. Defaults: surface `100,200,400`; volume `50,100,200,300` |
| `--repeats <n>` | Timed repetitions per case (default: `3`) |
| `--warmup <n>` | Warmup repetitions per case (default: `1`) |
| `--json <file>` | Write results as JSON |

The volume case uses `volumeGroups = {0}` and `splitHexahedra = true`, matching
`StaircaseMesherTest.fillsSphereAsSingleClosedUnitHexahedralVolume` (the 50³ case
additionally verifies the golden count of 7967 hexahedra). Every case verifies
that repeated runs produce identical element/coordinate counts.

For stable numbers, pin the process to a single core and use at least 5 repeats:

```bash
taskset -c 0 build/bin/tessellator_benchmarks --repeats 5 --warmup 1
```

`StaircaseMesher::getTimings()` reports wall-clock time per processing phase.
Times are medians over the timed repetitions, in seconds.

## Baseline

Machine: Intel Core i9-14900KF, 32 threads, Linux, Release (`-O3 -DNDEBUG`),
single core via `taskset -c 0`, 5 repeats, 1 warmup, captured 2026-10-06.

| Case | Total | slicing | collapsing | staircasing | removeOverlapped | compressSurfaces | volumeFiller | mergeAndClean | collapseNodes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| surface 100³ | 0.335 | 0.069 | 0.067 | 0.120 | 0.029 | 0.044 | – | 0.002 | 0.000 |
| surface 200³ | 1.220 | 0.208 | 0.231 | 0.461 | 0.120 | 0.175 | – | 0.009 | 0.001 |
| surface 400³ | 5.215 | 0.720 | 1.355 | 1.798 | 0.541 | 0.690 | – | 0.041 | 0.006 |
| volume 50³ | 0.115 | 0.027 | 0.024 | 0.038 | 0.008 | – | 0.010 | 0.004 | 0.001 |
| volume 100³ | 0.427 | 0.068 | 0.066 | 0.119 | 0.028 | – | 0.093 | 0.043 | 0.003 |
| volume 200³ | 2.592 | 0.209 | 0.233 | 0.454 | 0.118 | – | 1.008 | 0.504 | 0.041 |
| volume 300³ | 8.329 | 0.426 | 0.717 | 0.997 | 0.288 | – | 3.588 | 1.994 | 0.262 |

Raw baseline output: `results/baseline.txt`, `results/baseline.json`.

## After optimizations

Same machine and methodology, captured 2026-10-06 after the performance work on
`RedundancyCleaner`, `GridTools`, `Slicer`/`Collapser` paths, `Staircaser`,
`Compressor`, `VolumeFiller`, node collapsing, and the addition of
`utils/FlatHashMap.h`.

| Case | Total | Speedup | slicing | collapsing | staircasing | removeOverlapped | compressSurfaces | volumeFiller | mergeAndClean | collapseNodes |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| surface 100³ | 0.148 | 2.26× | 0.047 | 0.009 | 0.059 | 0.005 | 0.028 | – | 0.001 | 0.000 |
| surface 200³ | 0.503 | 2.42× | 0.136 | 0.032 | 0.201 | 0.021 | 0.111 | – | 0.004 | 0.001 |
| surface 400³ | 1.935 | 2.69× | 0.450 | 0.181 | 0.751 | 0.095 | 0.450 | – | 0.019 | 0.003 |
| volume 50³ | 0.044 | 2.61× | 0.017 | 0.004 | 0.018 | 0.002 | – | 0.003 | 0.000 | 0.000 |
| volume 100³ | 0.141 | 3.03× | 0.054 | 0.010 | 0.060 | 0.005 | – | 0.017 | 0.001 | 0.003 |
| volume 200³ | 0.593 | 4.37× | 0.164 | 0.028 | 0.198 | 0.020 | – | 0.163 | 0.014 | 0.025 |
| volume 300³ | 1.775 | 4.69× | 0.336 | 0.087 | 0.421 | 0.060 | – | 0.650 | 0.057 | 0.095 |

Phase values are medians and may not add up exactly to the total (unmeasured
sections and rounding). Raw output: `results/after.txt`, `results/after.json`.

Remaining hotspots at the largest sizes: staircasing (surface 400³),
compression and slicing (surface 400³), and `volumeFiller` (volume 300³).
