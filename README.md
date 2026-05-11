# Vanav-diel

Unified workspace for FFXI navmesh generation, routing, and visualization.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Options

- `BUILD_BIN_CONVERT` – OBJ to navmesh converter (default: ON)
- `BUILD_BIN_ROUTING_CLI` – zone routing CLI (default: ON)
- `BUILD_BIN_DEBUG_TOOL` – debug connectivity tool (default: ON)
- `BUILD_BIN_VISUALIZER` – navmesh visualizer (default: ON)
- `BUILD_UNIT_TESTS` – unit tests (default: ON)
- `VANAV_MSET_DIR` – path to navmesh directory (default: ./MSETs)

Example:

```bash
cmake -S . -B build -DBUILD_BIN_VISUALIZER=OFF -DBUILD_UNIT_TESTS=ON
```

## Binaries

- **vanav_convert** – OBJ to baked zone navmesh generator
- **vanav_query** – world and zone routing CLI
- **vanav_debug** – connectivity diagnostic tool
- **vanav_viewer** – 3D navmesh visualizer

## Tests

```bash
cmake --build build --target vanav_tests
./build/vanav_tests
```

Requires `BUILD_UNIT_TESTS=ON`.
