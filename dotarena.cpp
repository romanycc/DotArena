#include "dotarena.hpp"

PYBIND11_MODULE(_dotarena, m) {
  pybind11::class_<DotArena>(m, "DotArena")
      .def(pybind11::init<std::tuple<double, double, double>, double>())
      .def("add_circle", &DotArena::add_circle)
      .def("add_random_circles", &DotArena::add_random_circles)
      .def("checkCollisionBruteForce", &DotArena::checkCollisionBruteForce)
      .def("checkCollisionGrid1D", &DotArena::checkCollisionGrid1D)
      .def("step", &DotArena::step, pybind11::arg("dt"), pybind11::arg("method") = 2);

  pybind11::class_<Renderer>(m, "Renderer")
      .def(pybind11::init<int, int>(), pybind11::arg("width") = 800,
           pybind11::arg("height") = 800)
      .def(
          "renderToGif",
          [](Renderer &r, DotArena &sim, std::string filename, int frames,
             double dt) {
            auto step_func = [&sim, dt]() { sim.step(dt); };
            r.renderToGif(sim, step_func, filename, frames, dt);
          },
          pybind11::arg("sim"), pybind11::arg("filename"),
          pybind11::arg("frames"), pybind11::arg("dt"));
}
