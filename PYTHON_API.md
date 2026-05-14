# DotArena Python API Documentation

The `_dotarena` module is a high-performance, C++ backend physics engine for simulating 3D particle collisions. It is heavily optimized using Data-Oriented Design (SoA), ARM NEON SIMD vectorization, and Zero-Copy memory sharing with Python's NumPy.

---

## `_dotarena.DotArena`

The core physics simulation engine.

### `__init__(size: tuple[float, float, float], friction: float)`
Initializes a new physics arena.
* **`size`**: A 3-element tuple `(width, height, depth)` representing the total dimensions of the arena bounding box. The origin `(0,0,0)` is at the center of the arena.
* **`friction`**: A float representing the friction coefficient (e.g., `0.1`). Velocity is scaled by `max(0.0, 1.0 - friction * dt)` each step.

### `add_circle(id: int, radius: float, pos: tuple[float, float, float], dir: tuple[float, float, float]) -> None`
Manually adds a single circle/particle to the simulation.
* **`id`**: Unique integer identifier.
* **`radius`**: Radius of the particle.
* **`pos`**: Initial position `(x, y, z)`.
* **`dir`**: Initial velocity vector `(vx, vy, vz)`.

### `add_random_circles(count: int) -> None`
Populates the arena with `count` randomly generated circles. Useful for benchmarking and stress testing. 

### `step(dt: float, method: int = 2) -> None`
Advances the physics simulation by a single time step `dt`. Handles both movement (Euler integration) and collision resolution.
* **`dt`**: Delta time in seconds (e.g., `0.05`).
* **`method`**: The collision detection algorithm to use.
  * `0`: Brute Force $O(N^2)$ (SIMD Accelerated)
  * `2`: 1D Spatial Hash Grid $O(N)$ (SIMD Accelerated) - **Default and Recommended**

### `checkCollisionBruteForce() -> None`
Manually triggers the $O(N^2)$ Brute Force collision detection pass. Generally, you should call `step()` instead.

### `checkCollisionGrid1D() -> None`
Manually triggers the $O(N)$ 1D Spatial Hashing collision detection pass. Generally, you should call `step()` instead.

---

## NumPy Zero-Copy Properties (`DotArena`)

The `DotArena` class exposes its internal C++ arrays directly to Python via the Buffer Protocol. These properties return **NumPy Arrays** (`numpy.ndarray`) of type `float64`. 

> **Important:** These properties are **Zero-Copy**. Editing an element in Python (e.g., `da.px[0] = 100.0`) instantly and directly modifies the C++ physics engine's memory in $O(1)$ time.

* **`da.px`**: NumPy array of all X-positions.
* **`da.py`**: NumPy array of all Y-positions.
* **`da.pz`**: NumPy array of all Z-positions.
* **`da.vx`**: NumPy array of all X-velocities.
* **`da.vy`**: NumPy array of all Y-velocities.
* **`da.vz`**: NumPy array of all Z-velocities.
* **`da.radius`**: NumPy array of all radii.

---

## `_dotarena.Renderer`

A lightweight utility class for visualizing the physics simulation.

### `__init__(width: int = 800, height: int = 800)`
Initializes the renderer.
* **`width`**: Output image width in pixels.
* **`height`**: Output image height in pixels.

### `renderToGif(sim: DotArena, filename: str, frames: int, dt: float) -> None`
Automatically steps the simulation and renders the output to an animated GIF.
* **`sim`**: The `DotArena` instance to simulate and render.
* **`filename`**: The output path (e.g., `"gif/simulation.gif"`). Ensure the directory exists.
* **`frames`**: The total number of frames to simulate and record.
* **`dt`**: The time step duration per frame.

---

## Example Usage

```python
import _dotarena
import numpy as np

# 1. Create Engine
da = _dotarena.DotArena((1000.0, 1000.0, 1000.0), 0.1)

# 2. Add Entities
da.add_random_circles(5000)

# 3. Step Simulation
da.step(0.05, method=2)

# 4. Render Output
renderer = _dotarena.Renderer(400, 400)
renderer.renderToGif(da, "gif/output.gif", 100, 0.05)
```
