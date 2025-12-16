# Benchmarking and parity checks

The repository includes a simple benchmark/parity harness that compares the JavaScript solver with the C++/WASM build.

## Steps

1. Build the WASM module:
   ```
   npm run build:wasm
   ```
2. Run the comparison script:
   ```
   npm run compare
   ```

The script runs a handful of seeded random targets through both solvers, prints the average solve time for JS and WASM, and reports the maximum absolute error between resulting end‑effector positions. It exits non‑zero when the error exceeds the configurable threshold (default `1e-4`, override with `CCIK_MAX_ERROR`).

## Optimization notes

- The Emscripten build uses `-O3`, `-s WASM=1`, `-s MODULARIZE=1`, and `--bind`. Further improvements can come from enabling SIMD (`-msimd128`) when the host supports it and avoiding unnecessary heap allocations in the JS shim.
- The native C++ build can be compiled in `Release` with `-O3` for faster convergence and deterministic results.
- Preallocating chain descriptions and reusing solver instances avoids repeated heap traffic in both JS and WASM contexts.
