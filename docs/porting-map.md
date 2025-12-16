# Porting map: JavaScript IK core to C++/WASM

This document explains how the original JavaScript inverse kinematics solver maps onto the native C++ implementation and its Emscripten bindings.

## JavaScript sources

- `src/core/Frame.js` – base scene graph node with local/world transforms.
- `src/core/Link.js` – link node that can contain `Joint` children.
- `src/core/Joint.js` – joint node with DoF storage and limits.
- `src/core/Goal.js` – closure/goal target used during solving.
- `src/core/ChainSolver.js` – damped least-squares closed‑chain solver.
- `src/core/Solver.js` – orchestrates per‑chain solving and shared settings.
- `src/core/utils/*` – helper math routines, matrix pooling, SVD wrapper.
- `src/index.js` – public API surface.

## C++ equivalents

- `cpp/ccik/include/ccik/Types.h` – `Vec3` utility, tolerance constant, and enum describing a joint's degree of freedom.
- `cpp/ccik/include/ccik/Chain.h` – lightweight representation of a serial chain assembled from a JSON‑like description (array of joint specs with axis, length, limits, and rotation/translation mode). Responsible for forward kinematics and exposing joint/world positions.
- `cpp/ccik/include/ccik/IKSolver.h` – damped least‑squares solver that mirrors the behavior of `ChainSolver`/`Solver` for a single chain. Provides `setTarget(Vec3)`, `solve(int iterations)`, and `getPositions()` (returns world positions for each joint and the end effector).
- `cpp/ccik/src/Chain.cpp` and `cpp/ccik/src/IKSolver.cpp` – implementations of the above types using double precision and deterministic math.

## Bindings and JavaScript shim

- `cpp/ccik/bindings/embind.cpp` – Embind exposure of `Vec3`, `JointSpec`, `Chain`, `IKSolver`, and the `CCIK_TOLERANCE` constant. Accepts JS objects/arrays that mirror the C++ structs.
- `lib/ccik-wasm.js` – Promise‑based wrapper that loads `dist/ccik.js` (Emscripten MODULARIZE build) and re‑exports the WASM classes under the same names as the JS solver.

## Behavioral notes

- The C++ solver uses the same damped least‑squares strategy as the JS implementation but focuses on a single serial chain. Rotational joints use an axis vector and optional limits; translational joints move along the axis. All math uses `double` for deterministic parity.
- Link extension (`JointSpec.length`) offsets along the joint's local +X direction for rotational joints (so planar two‑link examples behave as expected) and along the joint axis for translational joints.
- Numeric parity is checked with a shared tolerance of `1e-6` (documented in code and tests). Randomized parity checks are provided in `scripts/compare-results.js`.
- The public API exposed to JS mirrors the original surface: create a chain from a description, set a target, call `solve(iterations)`, and read back positions.
