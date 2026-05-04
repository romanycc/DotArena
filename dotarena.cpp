#include "arena.hpp"
#include "circle.hpp"
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
using namespace std;
class DotArena{
    public:
        DotArena(tuple<double, double, double> size, double shrink_rate, double friction_coefficient) : arena(size, shrink_rate, friction_coefficient) {}
        void add_circle(int id, double r, tuple<double, double, double> pos, tuple<double, double, double> dir){
            circles.emplace_back(id, r, pos, dir);
        }
    private:
        vector<Circle> circles;
        Arena arena;
};
PYBIND11_MODULE(_dotarena, m) {
  pybind11::class_<DotArena>(m, "DotArena")
    .def(pybind11::init<tuple<double, double, double>, double, double>())
    .def("add_circle", &DotArena::add_circle);
}