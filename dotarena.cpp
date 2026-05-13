#include "arena.hpp"
#include "circle.hpp"
#include "renderer.hpp"
#include "vector3.hpp"
#include <algorithm>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <random>
using namespace std;
using Vector3 = std::tuple<double, double, double>;
class DotArena {
public:
  DotArena(tuple<double, double, double> size, double friction_coefficient)
      : arena(size, friction_coefficient) {}
  void add_circle(int id, double r, tuple<double, double, double> pos,
                  tuple<double, double, double> dir) {
    circles.emplace_back(id, r, pos, dir);
  }

  const std::vector<Circle> &getCircles() const { return circles; }
  const Arena &getArena() const { return arena; }

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

    int current_id = circles.size() + 1;
    for (int i = 0; i < count; ++i) {
      double r = dis_r(gen);
      // Keep balls fully inside the boundary at start
      double px = std::clamp(dis_x(gen), -sx + r, sx - r);
      double py = std::clamp(dis_y(gen), -sy + r, sy - r);
      double pz = std::clamp(dis_z(gen), -sz + r, sz - r);

      add_circle(current_id++, r, {px, py, pz},
                 {dis_v(gen), dis_v(gen), dis_v(gen)});
    }
  }
  bool isCollision(Circle &a, Circle &b) {
    Vector3 delta = sub(a.getPos(), b.getPos());
    double distSquare = dot(delta, delta);
    double radiusSum = a.getRadius() + b.getRadius();
    return distSquare <= radiusSum * radiusSum;
  }
  void resolveCollision(Circle &a, Circle &b) {
    Vector3 delta = sub(a.getPos(), b.getPos());
    double dist = sqrt(dot(delta, delta));
    Vector3 normal = {get<0>(delta) / dist, get<1>(delta) / dist,
                      get<2>(delta) / dist};
    // 速度在法向量上的分量
    Vector3 relativeV = sub(a.getDir(), b.getDir());
    double vAlongNormal = dot(relativeV, normal);
    // 如果速度已經是分開的方向，則不處理
    if (vAlongNormal > 0)
      return;

    // 計算質量 (mass = r^3)
    double rA = a.getRadius();
    double rB = b.getRadius();
    double massA = rA * rA * rA;
    double massB = rB * rB * rB;
    double inv_massA = 1.0 / massA;
    double inv_massB = 1.0 / massB;

    // 彈性碰撞衝量
    double restitution = 1.0;
    double impulseScalar =
        -(1 + restitution) * vAlongNormal / (inv_massA + inv_massB);

    // 更新速度
    a.setDir({
        get<0>(a.getDir()) + impulseScalar * inv_massA * get<0>(normal),
        get<1>(a.getDir()) + impulseScalar * inv_massA * get<1>(normal),
        get<2>(a.getDir()) + impulseScalar * inv_massA * get<2>(normal),
    });
    b.setDir({
        get<0>(b.getDir()) - impulseScalar * inv_massB * get<0>(normal),
        get<1>(b.getDir()) - impulseScalar * inv_massB * get<1>(normal),
        get<2>(b.getDir()) - impulseScalar * inv_massB * get<2>(normal),
    });
  }
  void checkCollisionBruteForce() {
    for (size_t i = 0; i < circles.size(); ++i) {
      for (size_t j = i + 1; j < circles.size(); ++j) {
        if (isCollision(circles[i], circles[j])) {
          resolveCollision(circles[i], circles[j]);
        }
      }
    }
  }

  void checkCollisionGrid() {
    double sx = get<0>(arena.getSize()) / 2.0;
    double sy = get<1>(arena.getSize()) / 2.0;
    double sz = get<2>(arena.getSize()) / 2.0;

    double cell_size = 50.0; // 2 * max_radius
    int grid_w = std::max(1, (int)std::ceil((2 * sx) / cell_size));
    int grid_h = std::max(1, (int)std::ceil((2 * sy) / cell_size));
    int grid_d = std::max(1, (int)std::ceil((2 * sz) / cell_size));

    std::vector<std::vector<int>> grid(grid_w * grid_h * grid_d);

    auto getCellIndex = [&](Vector3 pos) -> int {
      int cx = std::clamp((int)((get<0>(pos) + sx) / cell_size), 0, grid_w - 1);
      int cy = std::clamp((int)((get<1>(pos) + sy) / cell_size), 0, grid_h - 1);
      int cz = std::clamp((int)((get<2>(pos) + sz) / cell_size), 0, grid_d - 1);
      return cz * grid_w * grid_h + cy * grid_w + cx;
    };

    // Phase 1: Scatter
    for (size_t i = 0; i < circles.size(); ++i) {
      grid[getCellIndex(circles[i].getPos())].push_back(i);
    }

    // Phase 2: Gather
    for (size_t i = 0; i < circles.size(); ++i) {
      Vector3 pos = circles[i].getPos();
      int cx = std::clamp((int)((get<0>(pos) + sx) / cell_size), 0, grid_w - 1);
      int cy = std::clamp((int)((get<1>(pos) + sy) / cell_size), 0, grid_h - 1);
      int cz = std::clamp((int)((get<2>(pos) + sz) / cell_size), 0, grid_d - 1);

      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = cx + dx, ny = cy + dy, nz = cz + dz;
            if (nx >= 0 && nx < grid_w && ny >= 0 && ny < grid_h && nz >= 0 && nz < grid_d) {
              int neighbor_idx = nz * grid_w * grid_h + ny * grid_w + nx;
              for (int j : grid[neighbor_idx]) {
                if (i < (size_t)j) {
                  if (isCollision(circles[i], circles[j])) {
                    resolveCollision(circles[i], circles[j]);
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  // dt : delta time
  void step(double dt, bool use_grid = true) {
    // 1. Update positions based on velocity and apply friction
    double friction = arena.getFriction();
    for (auto &circle : circles) {
      Vector3 pos = circle.getPos();
      Vector3 dir = circle.getDir();

      // Apply friction (simple exponential decay of velocity)
      double frictionFactor = max(0.0, 1.0 - friction * dt);
      dir = {get<0>(dir) * frictionFactor, get<1>(dir) * frictionFactor,
             get<2>(dir) * frictionFactor};
      circle.setDir(dir);

      // Update position
      pos = {get<0>(pos) + get<0>(dir) * dt, get<1>(pos) + get<1>(dir) * dt,
             get<2>(pos) + get<2>(dir) * dt};

      // 3D Bounding Box Collision
      double sx = get<0>(arena.getSize()) / 2.0;
      double sy = get<1>(arena.getSize()) / 2.0;
      double sz = get<2>(arena.getSize()) / 2.0;
      double r = circle.getRadius();

      // Check X axis
      if (get<0>(pos) + r > sx) {
        get<0>(pos) = sx - r;
        get<0>(dir) *= -1.0;
      } else if (get<0>(pos) - r < -sx) {
        get<0>(pos) = -sx + r;
        get<0>(dir) *= -1.0;
      }

      // Check Y axis
      if (get<1>(pos) + r > sy) {
        get<1>(pos) = sy - r;
        get<1>(dir) *= -1.0;
      } else if (get<1>(pos) - r < -sy) {
        get<1>(pos) = -sy + r;
        get<1>(dir) *= -1.0;
      }

      // Check Z axis
      if (get<2>(pos) + r > sz) {
        get<2>(pos) = sz - r;
        get<2>(dir) *= -1.0;
      } else if (get<2>(pos) - r < -sz) {
        get<2>(pos) = -sz + r;
        get<2>(dir) *= -1.0;
      }

      circle.setPos(pos);
      circle.setDir(dir);
    }

    // 2. Resolve collisions
    if (use_grid) {
      checkCollisionGrid();
    } else {
      checkCollisionBruteForce();
    }
  }

private:
  std::vector<Circle> circles;
  Arena arena;
};
PYBIND11_MODULE(_dotarena, m) {
  pybind11::class_<DotArena>(m, "DotArena")
      .def(pybind11::init<tuple<double, double, double>, double>())
      .def("add_circle", &DotArena::add_circle)
      .def("add_random_circles", &DotArena::add_random_circles)
      .def("isCollision", &DotArena::isCollision)
      .def("resolveCollision", &DotArena::resolveCollision)
      .def("checkCollisionBruteForce", &DotArena::checkCollisionBruteForce)
      .def("checkCollisionGrid", &DotArena::checkCollisionGrid)
      .def("step", &DotArena::step, pybind11::arg("dt"), pybind11::arg("use_grid") = true);

  pybind11::class_<Renderer>(m, "Renderer")
      .def(pybind11::init<int, int>(), pybind11::arg("width") = 800,
           pybind11::arg("height") = 800)
      .def(
          "renderToGif",
          [](Renderer &r, DotArena &sim, std::string filename, int frames,
             double dt) {
            auto step_func = [&sim, dt]() { sim.step(dt); };
            r.renderToGif(sim.getArena(), sim.getCircles(), step_func, filename,
                          frames, dt);
          },
          pybind11::arg("sim"), pybind11::arg("filename"),
          pybind11::arg("frames"), pybind11::arg("dt"));
  pybind11::class_<Circle>(m, "Circle")
      .def(pybind11::init<int, double, tuple<double, double, double>,
                          tuple<double, double, double>>())
      .def("getPos", &Circle::getPos)
      .def("getDir", &Circle::getDir)
      .def("getRadius", &Circle::getRadius);
}
