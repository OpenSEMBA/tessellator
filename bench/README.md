# Tessellator benchmarks

Standalone, dependency-free benchmarks for `meshlib::meshers::StaircaseMesher`.
The sphere case is loaded from `testData/cases/sphere/sphere.stl`; grid sizes are
parameterized per axis.

## Build

Benchmarks are disabled by default and require the app (the sphere STL is loaded
through `vtkIO`):

```bash
cmake --preset gnu -S . -B build -DTESSELLATOR_ENABLE_BENCHMARKS=ON
cmake --build build -j --target tessellator_benchmarks
```

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
