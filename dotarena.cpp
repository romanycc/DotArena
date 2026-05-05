#include "arena.hpp"
#include "circle.hpp"
#include "vector3.hpp"
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
      .def("checkCollision", &DotArena::checkCollision);
  pybind11::class_<Circle>(m, "Circle")
      .def(pybind11::init<int, double, tuple<double, double, double>,
                          tuple<double, double, double>>())
      .def("getPos", &Circle::getPos)
      .def("getDir", &Circle::getDir)
      .def("getRadius", &Circle::getRadius);
}
