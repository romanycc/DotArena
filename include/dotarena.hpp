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

class DotArena {
public:
  DotArena(tuple<double, double, double> size, double friction_coefficient)
      : arena(size, friction_coefficient), num_circles(0) {}

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

  const Arena &getArena() const { return arena; }
  int getNumCircles() const { return num_circles; }
  
  const std::vector<double>& getPx() const { return px; }
  const std::vector<double>& getPy() const { return py; }
  const std::vector<double>& getPz() const { return pz; }
  const std::vector<double>& getRadius() const { return radius; }

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

  bool isCollision(size_t i, size_t j) {
    double dx = px[i] - px[j];
    double dy = py[i] - py[j];
    double dz = pz[i] - pz[j];
    double distSquare = dx*dx + dy*dy + dz*dz;
    double radiusSum = radius[i] + radius[j];
    return distSquare <= radiusSum * radiusSum;
  }

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

  void checkCollisionBruteForce() {
#if defined(__ARM_NEON)
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      float64x2_t pxi = vdupq_n_f64(px[i]);
      float64x2_t pyi = vdupq_n_f64(py[i]);
      float64x2_t pzi = vdupq_n_f64(pz[i]);
      float64x2_t ri  = vdupq_n_f64(radius[i]);

      size_t j = i + 1;
      // Process 2 circles at a time using 128-bit NEON registers
      for (; j + 1 < (size_t)num_circles; j += 2) {
        float64x2_t pxj = vld1q_f64(&px[j]);
        float64x2_t pyj = vld1q_f64(&py[j]);
        float64x2_t pzj = vld1q_f64(&pz[j]);
        float64x2_t rj  = vld1q_f64(&radius[j]);

        float64x2_t dx = vsubq_f64(pxi, pxj);
        float64x2_t dy = vsubq_f64(pyi, pyj);
        float64x2_t dz = vsubq_f64(pzi, pzj);

        float64x2_t distSq = vmulq_f64(dx, dx);
        distSq = vfmaq_f64(distSq, dy, dy); // Fused multiply-add
        distSq = vfmaq_f64(distSq, dz, dz);

        float64x2_t rSum = vaddq_f64(ri, rj);
        float64x2_t rSumSq = vmulq_f64(rSum, rSum);

        uint64x2_t cmp = vcleq_f64(distSq, rSumSq);

        if (vgetq_lane_u64(cmp, 0)) {
          resolveCollision(i, j);
        }
        if (vgetq_lane_u64(cmp, 1)) {
          resolveCollision(i, j + 1);
        }
      }
      // Scalar fallback for remainder
      for (; j < (size_t)num_circles; ++j) {
        if (isCollision(i, j)) {
          resolveCollision(i, j);
        }
      }
    }
#else
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      for (size_t j = i + 1; j < (size_t)num_circles; ++j) {
        if (isCollision(i, j)) {
          resolveCollision(i, j);
        }
      }
    }
#endif
  }

  void checkCollisionGrid1D() {
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

    auto getCellIndex = [&](size_t i) -> int {
      int cx = std::clamp((int)((px[i] + sx) / cell_size), 0, grid_w - 1);
      int cy = std::clamp((int)((py[i] + sy) / cell_size), 0, grid_h - 1);
      int cz = std::clamp((int)((pz[i] + sz) / cell_size), 0, grid_d - 1);
      return cz * grid_w * grid_h + cy * grid_w + cx;
    };

    #pragma omp simd
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      int cell_idx = getCellIndex(i);
      circle_next[i] = grid_head[cell_idx];
      grid_head[cell_idx] = i;
    }

    int neighbor_buffer[512]; // Stack allocated raw array

    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      int cx = std::clamp((int)((px[i] + sx) / cell_size), 0, grid_w - 1);
      int cy = std::clamp((int)((py[i] + sy) / cell_size), 0, grid_h - 1);
      int cz = std::clamp((int)((pz[i] + sz) / cell_size), 0, grid_d - 1);

      size_t buf_size = 0;

      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = cx + dx, ny = cy + dy, nz = cz + dz;
            if (nx >= 0 && nx < grid_w && ny >= 0 && ny < grid_h && nz >= 0 && nz < grid_d) {
              int neighbor_idx = nz * grid_w * grid_h + ny * grid_w + nx;
              int j = grid_head[neighbor_idx];
              while (j != -1) {
                if (i < (size_t)j) {
                  neighbor_buffer[buf_size++] = j;
                  if (buf_size >= 512) break; // Safety limit
                }
                j = circle_next[j];
              }
            }
          }
        }
      }

      size_t k = 0;
#if defined(__ARM_NEON)
      float64x2_t pxi = vdupq_n_f64(px[i]);
      float64x2_t pyi = vdupq_n_f64(py[i]);
      float64x2_t pzi = vdupq_n_f64(pz[i]);
      float64x2_t ri  = vdupq_n_f64(radius[i]);

      for (; k + 1 < buf_size; k += 2) {
        int j0 = neighbor_buffer[k];
        int j1 = neighbor_buffer[k+1];

        // Gather from SoA
        double px_arr[2] = {px[j0], px[j1]};
        double py_arr[2] = {py[j0], py[j1]};
        double pz_arr[2] = {pz[j0], pz[j1]};
        double r_arr[2]  = {radius[j0], radius[j1]};

        float64x2_t pxj = vld1q_f64(px_arr);
        float64x2_t pyj = vld1q_f64(py_arr);
        float64x2_t pzj = vld1q_f64(pz_arr);
        float64x2_t rj  = vld1q_f64(r_arr);

        float64x2_t dx = vsubq_f64(pxi, pxj);
        float64x2_t dy = vsubq_f64(pyi, pyj);
        float64x2_t dz = vsubq_f64(pzi, pzj);

        float64x2_t distSq = vmulq_f64(dx, dx);
        distSq = vfmaq_f64(distSq, dy, dy); // Fused multiply-add
        distSq = vfmaq_f64(distSq, dz, dz);

        float64x2_t rSum = vaddq_f64(ri, rj);
        float64x2_t rSumSq = vmulq_f64(rSum, rSum);

        uint64x2_t cmp = vcleq_f64(distSq, rSumSq);

        if (vgetq_lane_u64(cmp, 0)) resolveCollision(i, j0);
        if (vgetq_lane_u64(cmp, 1)) resolveCollision(i, j1);
      }
#endif
      // Scalar remainder
      for (; k < buf_size; ++k) {
        int j = neighbor_buffer[k];
        if (isCollision(i, j)) {
          resolveCollision(i, j);
        }
      }
    }
  }

  void step(double dt, int method = 2) {
    double friction = arena.getFriction();
    double frictionFactor = max(0.0, 1.0 - friction * dt);

    double sx = get<0>(arena.getSize()) / 2.0;
    double sy = get<1>(arena.getSize()) / 2.0;
    double sz = get<2>(arena.getSize()) / 2.0;

    #pragma omp simd
    for (size_t i = 0; i < (size_t)num_circles; ++i) {
      vx[i] *= frictionFactor;
      vy[i] *= frictionFactor;
      vz[i] *= frictionFactor;

      px[i] += vx[i] * dt;
      py[i] += vy[i] * dt;
      pz[i] += vz[i] * dt;

      double r = radius[i];

      if (px[i] + r > sx) { px[i] = sx - r; vx[i] *= -1.0; }
      else if (px[i] - r < -sx) { px[i] = -sx + r; vx[i] *= -1.0; }

      if (py[i] + r > sy) { py[i] = sy - r; vy[i] *= -1.0; }
      else if (py[i] - r < -sy) { py[i] = -sy + r; vy[i] *= -1.0; }

      if (pz[i] + r > sz) { pz[i] = sz - r; vz[i] *= -1.0; }
      else if (pz[i] - r < -sz) { pz[i] = -sz + r; vz[i] *= -1.0; }
    }

    if (method == 2) {
      checkCollisionGrid1D();
    } else {
      checkCollisionBruteForce(); // Removed 2D grid
    }
  }

private:
  Arena arena;
  int num_circles;
  std::vector<int> ids;
  std::vector<double> px, py, pz;
  std::vector<double> vx, vy, vz;
  std::vector<double> radius;
  std::vector<int> grid_head;
  std::vector<int> circle_next;
};

#endif
