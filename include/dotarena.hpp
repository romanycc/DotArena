#ifndef DOTARENA_HPP
#define DOTARENA_HPP

#include "arena.hpp"
#include "renderer.hpp"
#include <algorithm>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <random>

#if defined(__ARM_NEON)
#include <arm_neon.h>
#endif
using namespace std;
using Vector3 = std::tuple<double, double, double>;

template <typename T, std::size_t Alignment>
struct AlignedAllocator {
    using value_type = T;

    AlignedAllocator() noexcept = default;
    template <typename U> AlignedAllocator(const AlignedAllocator<U, Alignment>&) noexcept {}

    template <typename U>
    struct rebind {
        using other = AlignedAllocator<U, Alignment>;
    };

    T* allocate(std::size_t n) {
        if (n == 0) return nullptr;
        void* ptr = nullptr;
        if (posix_memalign(&ptr, Alignment, n * sizeof(T)) != 0) {
            throw std::bad_alloc();
        }
        return static_cast<T*>(ptr);
    }

    void deallocate(T* p, std::size_t) noexcept {
        free(p);
    }
};

template <typename T, typename U, std::size_t Alignment>
bool operator==(const AlignedAllocator<T, Alignment>&, const AlignedAllocator<U, Alignment>&) { return true; }
template <typename T, typename U, std::size_t Alignment>
bool operator!=(const AlignedAllocator<T, Alignment>&, const AlignedAllocator<U, Alignment>&) { return false; }

using DoubleVector = std::vector<double, AlignedAllocator<double, 64>>;

/**
 * @class DotArena
 * @brief High-performance 3D particle simulation engine core.
 * 
 * Manages particles using a Structure of Arrays (SoA) memory model, handles kinematics integration,
 * environment interactions, and collision checks utilizing NEON SIMD and a 1D spatial hash grid.
 */
class DotArena {
public:
  /**
   * @brief Constructs a new DotArena physics engine.
     * @param size Tuple defining the 3D dimensions (width, height, depth) of the boundary box.
     * @param friction_coefficient Friction damping factor applied dynamically each time step.
     */
  DotArena(tuple<double, double, double> size, double friction_coefficient)
      : arena(size, friction_coefficient), num_circles(0) {}

  /**
   * @brief Manually appends a single particle to the physics simulation.
   * @param id Unique identifier.
   * @param r Radius of the circle.
   * @param pos 3D tuple (x, y, z) for the starting position.
   * @param dir 3D tuple (vx, vy, vz) for the starting velocity vector.
   */
  void add_circle(int id, double r, tuple<double, double, double> pos,
                  tuple<double, double, double> dir) {
    ids.push_back(id);
    radius.push_back(r);
    px.push_back(get<0>(pos));
    py.push_back(get<1>(pos));
    pz.push_back(get<2>(pos));
    vx.push_back(get<0>(dir));
    vy.push_back(get<1>(dir));
    vz.push_back(get<2>(dir));
    num_circles++;
  }

  /**
   * @brief Gets a read-only reference to the simulation boundaries/parameters.
   */
  const Arena &getArena() const { return arena; }

  /**
   * @brief Gets the total count of active particles in the simulation.
   */
  int getNumCircles() const { return num_circles; }
  
  const DoubleVector& getPx() const { return px; }
  const DoubleVector& getPy() const { return py; }
  const DoubleVector& getPz() const { return pz; }
  const DoubleVector& getVx() const { return vx; }
  const DoubleVector& getVy() const { return vy; }
  const DoubleVector& getVz() const { return vz; }
  const DoubleVector& getRadius() const { return radius; }

  /**
   * @brief Populates the simulation with a specific count of randomly placed particles.
   * @param count Number of random circles to generate.
   */
  void add_random_circles(int count) {
    std::random_device rd;
    std::mt19937 gen(rd());

    double sx = get<0>(arena.getSize()) / 2.0;
    double sy = get<1>(arena.getSize()) / 2.0;
    double sz = get<2>(arena.getSize()) / 2.0;

    std::uniform_real_distribution<> dis_x(-sx, sx);
    std::uniform_real_distribution<> dis_y(-sy, sy);
    std::uniform_real_distribution<> dis_z(-sz, sz);
    std::uniform_real_distribution<> dis_v(-100.0, 100.0);
    std::uniform_real_distribution<> dis_r(5.0, 25.0);

    int current_id = num_circles + 1;
    for (int i = 0; i < count; ++i) {
      double r = dis_r(gen);
      double px_val = std::clamp(dis_x(gen), -sx + r, sx - r);
      double py_val = std::clamp(dis_y(gen), -sy + r, sy - r);
      double pz_val = std::clamp(dis_z(gen), -sz + r, sz - r);

      add_circle(current_id++, r, {px_val, py_val, pz_val},
                 {dis_v(gen), dis_v(gen), dis_v(gen)});
    }
  }

  /**
   * @brief Checks if two particles overlap in 3D space.
   * @param i First particle index.
   * @param j Second particle index.
   * @return True if particles intersect, false otherwise.
   */
  bool isCollision(size_t i, size_t j) {
    double dx = px[i] - px[j];
    double dy = py[i] - py[j];
    double dz = pz[i] - pz[j];
    double distSquare = dx*dx + dy*dy + dz*dz;
    double radiusSum = radius[i] + radius[j];
    return distSquare <= radiusSum * radiusSum;
  }

  /**
   * @brief Resolves elastic collision velocities between two particles using momentum conservation.
   * @param i First particle index.
   * @param j Second particle index.
   */
  void resolveCollision(size_t i, size_t j) {
    double dx = px[i] - px[j];
    double dy = py[i] - py[j];
    double dz = pz[i] - pz[j];
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dist == 0.0) return; // Prevent division by zero

    double nx = dx / dist;
    double ny = dy / dist;
    double nz = dz / dist;

    double rvx = vx[i] - vx[j];
    double rvy = vy[i] - vy[j];
    double rvz = vz[i] - vz[j];

    double vAlongNormal = rvx*nx + rvy*ny + rvz*nz;

    if (vAlongNormal > 0)
      return;

    double rA = radius[i];
    double rB = radius[j];
    double massA = rA * rA * rA;
    double massB = rB * rB * rB;
    double inv_massA = 1.0 / massA;
    double inv_massB = 1.0 / massB;

    double restitution = 1.0;
    double impulseScalar = -(1 + restitution) * vAlongNormal / (inv_massA + inv_massB);

    vx[i] += impulseScalar * inv_massA * nx;
    vy[i] += impulseScalar * inv_massA * ny;
    vz[i] += impulseScalar * inv_massA * nz;

    vx[j] -= impulseScalar * inv_massB * nx;
    vy[j] -= impulseScalar * inv_massB * ny;
    vz[j] -= impulseScalar * inv_massB * nz;
  }

  /**
   * @brief O(N^2) collision detection loop with ARM NEON SIMD optimization.
   */
  void checkCollisionBruteForce() {
#if defined(__ARM_NEON)
    // -------------------------------------------------------------------------
    // ARM NEON SIMD Accelerated Narrow-Phase Collision Check
    // -------------------------------------------------------------------------
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      // Duplicate particle i parameters into NEON float64x2 vector registers
      float64x2_t pxi = vdupq_n_f64(px[i]);
      float64x2_t pyi = vdupq_n_f64(py[i]);
      float64x2_t pzi = vdupq_n_f64(pz[i]);
      float64x2_t ri  = vdupq_n_f64(radius[i]);

      size_t j = i + 1;
      // Process 2 neighbor circles in parallel using 128-bit operations
      for (; j + 1 < (size_t)num_circles; j += 2) {
        float64x2_t pxj = vld1q_f64(&px[j]);
        float64x2_t pyj = vld1q_f64(&py[j]);
        float64x2_t pzj = vld1q_f64(&pz[j]);
        float64x2_t rj  = vld1q_f64(&radius[j]);

        // Calculate differences
        float64x2_t dx = vsubq_f64(pxi, pxj);
        float64x2_t dy = vsubq_f64(pyi, pyj);
        float64x2_t dz = vsubq_f64(pzi, pzj);

        // Fused multiply-add: distSq = dx^2 + dy^2 + dz^2
        float64x2_t distSq = vmulq_f64(dx, dx);
        distSq = vfmaq_f64(distSq, dy, dy);
        distSq = vfmaq_f64(distSq, dz, dz);

        // Calculate (r_i + r_j)^2
        float64x2_t rSum = vaddq_f64(ri, rj);
        float64x2_t rSumSq = vmulq_f64(rSum, rSum);

        // Vector comparison: distSq <= rSumSq
        uint64x2_t cmp = vcleq_f64(distSq, rSumSq);

        // Resolve collisions conditionally depending on lanes
        if (vgetq_lane_u64(cmp, 0)) {
          resolveCollision(i, j);
        }
        if (vgetq_lane_u64(cmp, 1)) {
          resolveCollision(i, j + 1);
        }
      }
      // Scalar fallback for remaining odd particle
      for (; j < (size_t)num_circles; ++j) {
        if (isCollision(i, j)) {
          resolveCollision(i, j);
        }
      }
    }
#else
    // -------------------------------------------------------------------------
    // Scalar C++ Fallback Loop (x86_64, non-ARM environments)
    // -------------------------------------------------------------------------
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      for (size_t j = i + 1; j < (size_t)num_circles; ++j) {
        if (isCollision(i, j)) {
          resolveCollision(i, j);
        }
      }
    }
#endif
  }

  /**
   * @brief O(N) Broad-Phase 1D Spatial Hash Grid collision detection solver.
   */
  void checkCollisionGrid1D() {
    // =========================================================================
    // SECTION 1: Grid Dimensions and Memory Resizing
    // =========================================================================
    double sx = get<0>(arena.getSize()) / 2.0;
    double sy = get<1>(arena.getSize()) / 2.0;
    double sz = get<2>(arena.getSize()) / 2.0;

    double cell_size = 50.0;
    int grid_w = std::max(1, (int)std::ceil((2 * sx) / cell_size));
    int grid_h = std::max(1, (int)std::ceil((2 * sy) / cell_size));
    int grid_d = std::max(1, (int)std::ceil((2 * sz) / cell_size));

    int total_cells = grid_w * grid_h * grid_d;
    if ((int)grid_head.size() != total_cells) {
      grid_head.resize(total_cells);
    }
    grid_head.assign(total_cells, -1);

    if (circle_next.size() < (size_t)num_circles) {
      circle_next.resize(num_circles);
    }

    // =========================================================================
    // SECTION 2: Cell Hashing and Linked List Generation
    // =========================================================================
    auto getCellIndex = [&](size_t i) -> int {
      int cx = std::clamp((int)((px[i] + sx) / cell_size), 0, grid_w - 1);
      int cy = std::clamp((int)((py[i] + sy) / cell_size), 0, grid_h - 1);
      int cz = std::clamp((int)((pz[i] + sz) / cell_size), 0, grid_d - 1);
      return cz * grid_w * grid_h + cy * grid_w + cx;
    };

    // Insert particles into grid cells linked list without dynamic allocations
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      int cell_idx = getCellIndex(i);
      circle_next[i] = grid_head[cell_idx];
      grid_head[cell_idx] = i;
    }

    int neighbor_buffer[512]; // Stack-allocated raw array buffer for candidate queries

    // =========================================================================
    // SECTION 3: Broad-Phase Neighbor Query
    // =========================================================================
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      int cx = std::clamp((int)((px[i] + sx) / cell_size), 0, grid_w - 1);
      int cy = std::clamp((int)((py[i] + sy) / cell_size), 0, grid_h - 1);
      int cz = std::clamp((int)((pz[i] + sz) / cell_size), 0, grid_d - 1);

      size_t buf_size = 0;

      // Scan 27-neighbor cells surrounding the current cell
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = cx + dx, ny = cy + dy, nz = cz + dz;
            if (nx >= 0 && nx < grid_w && ny >= 0 && ny < grid_h && nz >= 0 && nz < grid_d) {
              int neighbor_idx = nz * grid_w * grid_h + ny * grid_w + nx;
              int j = grid_head[neighbor_idx];
              while (j != -1) {
                // Assert strict ordering i < j to prevent duplicate pair checking
                if (i < (size_t)j) {
                  neighbor_buffer[buf_size++] = j;
                  if (buf_size >= 512) break; // Buffer size guard
                }
                j = circle_next[j];
              }
            }
          }
        }
      }

      // =========================================================================
      // SECTION 4: Narrow-Phase Collision Checking (NEON / Scalar fallback)
      // =========================================================================
      size_t k = 0;
#if defined(__ARM_NEON)
      // ARM NEON SIMD Accelerated check for grid neighbors
      float64x2_t pxi = vdupq_n_f64(px[i]);
      float64x2_t pyi = vdupq_n_f64(py[i]);
      float64x2_t pzi = vdupq_n_f64(pz[i]);
      float64x2_t ri  = vdupq_n_f64(radius[i]);

      for (; k + 1 < buf_size; k += 2) {
        int j0 = neighbor_buffer[k];
        int j1 = neighbor_buffer[k+1];

        // Gather coordinates from SoA arrays
        double px_arr[2] = {px[j0], px[j1]};
        double py_arr[2] = {py[j0], py[j1]};
        double pz_arr[2] = {pz[j0], pz[j1]};
        double r_arr[2]  = {radius[j0], radius[j1]};

        float64x2_t pxj = vld1q_f64(px_arr);
        float64x2_t pyj = vld1q_f64(py_arr);
        float64x2_t pzj = vld1q_f64(pz_arr);
        float64x2_t rj  = vld1q_f64(r_arr);

        // Vector calculations
        float64x2_t dx = vsubq_f64(pxi, pxj);
        float64x2_t dy = vsubq_f64(pyi, pyj);
        float64x2_t dz = vsubq_f64(pzi, pzj);

        float64x2_t distSq = vmulq_f64(dx, dx);
        distSq = vfmaq_f64(distSq, dy, dy);
        distSq = vfmaq_f64(distSq, dz, dz);

        float64x2_t rSum = vaddq_f64(ri, rj);
        float64x2_t rSumSq = vmulq_f64(rSum, rSum);

        uint64x2_t cmp = vcleq_f64(distSq, rSumSq);

        if (vgetq_lane_u64(cmp, 0)) resolveCollision(i, j0);
        if (vgetq_lane_u64(cmp, 1)) resolveCollision(i, j1);
      }
#endif
      // Scalar remainder loop (handles odd numbers or non-ARM builds)
      for (; k < buf_size; ++k) {
        int j = neighbor_buffer[k];
        if (isCollision(i, j)) {
          resolveCollision(i, j);
        }
      }
    }
  }

  /**
   * @brief Advances the physics engine state by dt.
   * @param dt Elapsed time step.
   * @param method Collision algorithm to employ (0 = Brute Force, 2 = 1D Grid).
   */
  void step(double dt, int method = 2) {
    // =========================================================================
    // PART A: Integration, Velocity Damping, and Boundary Clamping
    // =========================================================================
    double friction = arena.getFriction();
    double frictionFactor = max(0.0, 1.0 - friction * dt);

    double sx = get<0>(arena.getSize()) / 2.0;
    double sy = get<1>(arena.getSize()) / 2.0;
    double sz = get<2>(arena.getSize()) / 2.0;

    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      // 1. Friction Velocity Decay
      vx[i] *= frictionFactor;
      vy[i] *= frictionFactor;
      vz[i] *= frictionFactor;

      // 2. Position Euler Integration
      px[i] += vx[i] * dt;
      py[i] += vy[i] * dt;
      pz[i] += vz[i] * dt;

      double r = radius[i];

      // 3. Boundary Wall Collision Resolution & Clamping
      if (px[i] + r > sx) { px[i] = sx - r; vx[i] *= -1.0; }
      else if (px[i] - r < -sx) { px[i] = -sx + r; vx[i] *= -1.0; }

      if (py[i] + r > sy) { py[i] = sy - r; vy[i] *= -1.0; }
      else if (py[i] - r < -sy) { py[i] = -sy + r; vy[i] *= -1.0; }

      if (pz[i] + r > sz) { pz[i] = sz - r; vz[i] *= -1.0; }
      else if (pz[i] - r < -sz) { pz[i] = -sz + r; vz[i] *= -1.0; }
    }

    // =========================================================================
    // PART B: Collision Resolution Solver
    // =========================================================================
    if (method == 2) {
      checkCollisionGrid1D();
    } else {
      checkCollisionBruteForce();
    }
  }

private:
  Arena arena;                  ///< Env parameters (boundary sizes, friction constant).
  int num_circles;              ///< Active circle counts in simulation.
  std::vector<int> ids;         ///< Unique circle identification list.
  DoubleVector px, py, pz;      ///< Cache-aligned SoA position coordinates vector.
  DoubleVector vx, vy, vz;      ///< Cache-aligned SoA velocity vector.
  DoubleVector radius;          ///< Cache-aligned SoA circle radius vector.
  std::vector<int> grid_head;   ///< Spatial hash grid flat lookup heads list.
  std::vector<int> circle_next; ///< Spatial hash grid LinkedList next-pointing offsets.
};

#endif
