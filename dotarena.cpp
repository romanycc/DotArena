#include "arena.hpp"
#include "circle.hpp"
#include "gif.h"
#include "vector3.hpp"
#include <algorithm>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
using namespace std;
using Vector3 = std::tuple<double, double, double>;
class DotArena {
public:
  DotArena(tuple<double, double, double> size, double shrink_rate,
           double friction_coefficient)
      : arena(size, shrink_rate, friction_coefficient) {}
  void add_circle(int id, double r, tuple<double, double, double> pos,
                  tuple<double, double, double> dir) {
    circles.emplace_back(id, r, pos, dir);
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
    // 簡單彈性碰撞
    double restitution = 1.0;
    double impulseScalar = -(1 + restitution) * vAlongNormal;
    impulseScalar /= 2;
    // 更新速度
    a.setDir({
        get<0>(a.getDir()) + impulseScalar * get<0>(normal),
        get<1>(a.getDir()) + impulseScalar * get<1>(normal),
        get<2>(a.getDir()) + impulseScalar * get<2>(normal),
    });
    b.setDir({
        get<0>(b.getDir()) - impulseScalar * get<0>(normal),
        get<1>(b.getDir()) - impulseScalar * get<1>(normal),
        get<2>(b.getDir()) - impulseScalar * get<2>(normal),
    });
  }
  void checkCollision() {
    for (size_t i = 0; i < circles.size(); ++i) {
      for (size_t j = i + 1; j < circles.size(); ++j) {
        if (isCollision(circles[i], circles[j])) {
          resolveCollision(circles[i], circles[j]);
        }
      }
    }
  }
  // dt : delta time
  void step(double dt) {
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
      circle.setPos(pos);
    }

    // 2. Resolve collisions
    checkCollision();
  }

  void renderToGif(std::string filename, int frames, double dt, int width = 800,
                   int height = 800) {
    int delay = std::max(2, (int)(dt * 100)); // delay in hundredths of a second
    GifWriter g;
    GifBegin(&g, filename.c_str(), width, height, delay);
    // 焦距：影響透視感，數值越大透視感越弱（變像平行的），越小則廣角感越強
    double focal_length = 500.0;
    // 設定光源方向，這是一個單位化向量，指向左上方前方
    Vector3 light_dir = {0.577, -0.577, -0.577};
    for (int f = 0; f < frames; ++f) {
      std::vector<uint8_t> image(width * height * 4, 0); // black background

      // Sort circles by Z descending (Painter's algorithm: further objects
      // drawn first)
      // watch from z axis
      std::vector<Circle> sorted_circles = circles;
      std::sort(sorted_circles.begin(), sorted_circles.end(),
                [](Circle &a, Circle &b) {
                  return std::get<2>(a.getPos()) > std::get<2>(b.getPos());
                });

      for (auto &circle : sorted_circles) {
        Vector3 pos = circle.getPos();
        double z = std::get<2>(pos);

        // Avoid division by zero or negative focal depth
        if (focal_length + z <= 0.1)
          continue;
        // 讓遠處的球看起來比近處的球小
        double scale = focal_length / (focal_length + z);

        // Project 3D to 2D
        // 座標轉換
        double cx = std::get<0>(pos) * scale + width / 2.0;
        double cy = std::get<1>(pos) * scale + height / 2.0;
        double r = circle.getRadius() * scale;
        // bb of the circle
        // 這顆球只可能出現在這個小格子裡，外面的區域你完全不用理會。
        int minX = std::max(0, (int)(cx - r));
        int maxX = std::min(width - 1, (int)(cx + r));
        int minY = std::max(0, (int)(cy - r));
        int maxY = std::min(height - 1, (int)(cy + r));

        for (int y = minY; y <= maxY; ++y) {
          for (int x = minX; x <= maxX; ++x) {
            double dx = x - cx;
            double dy = y - cy;
            double distSq = dx * dx + dy * dy;
            // is the pixel in the circle ?
            if (distSq <= r * r) {
              // Calculate normal for Fake 3D Shading
              // 知道圓圈內每一個點的「朝向」
              // nz = 1, 表面正對著你
              // nz = 0, 表面側對著你
              double nx = dx / r;
              double ny = dy / r;
              double nz = -std::sqrt(std::max(
                  0.0, 1.0 - nx * nx - ny * ny)); // -Z is towards camera

              // Lighting calculation
              // 表面朝向 · 光源方向
              // 如果表面點「正對著」光源，內積結果會接近 1（最亮）。
              // 如果表面點與光源垂直（光擦身而過），內積結果是 0。
              double diffuse = std::max(0.0, nx * std::get<0>(light_dir) +
                                                 ny * std::get<1>(light_dir) +
                                                 nz * std::get<2>(light_dir));
              // 環境光
              double ambient = 0.3;
              // 總強度
              double intensity = std::min(1.0, ambient + diffuse);

              // Base color: nice energetic orange/red
              int r_col = std::min(255, (int)(255 * intensity));
              int g_col = std::min(255, (int)(100 * intensity));
              int b_col = std::min(255, (int)(50 * intensity));

              int index = (y * width + x) * 4;
              image[index + 0] = r_col;
              image[index + 1] = g_col;
              image[index + 2] = b_col;
              image[index + 3] = 255;
            }
          }
        }
      }
      GifWriteFrame(&g, image.data(), width, height, delay);
      step(dt); // Advance the simulation!
    }
    GifEnd(&g);
  }

private:
  vector<Circle> circles;
  Arena arena;
};
PYBIND11_MODULE(_dotarena, m) {
  pybind11::class_<DotArena>(m, "DotArena")
      .def(pybind11::init<tuple<double, double, double>, double, double>())
      .def("add_circle", &DotArena::add_circle)
      .def("isCollision", &DotArena::isCollision)
      .def("resolveCollision", &DotArena::resolveCollision)
      .def("checkCollision", &DotArena::checkCollision)
      .def("step", &DotArena::step)
      .def("renderToGif", &DotArena::renderToGif, pybind11::arg("filename"),
           pybind11::arg("frames"), pybind11::arg("dt"),
           pybind11::arg("width") = 800, pybind11::arg("height") = 800);
  pybind11::class_<Circle>(m, "Circle")
      .def(pybind11::init<int, double, tuple<double, double, double>,
                          tuple<double, double, double>>())
      .def("getPos", &Circle::getPos)
      .def("getDir", &Circle::getDir)
      .def("getRadius", &Circle::getRadius);
}
