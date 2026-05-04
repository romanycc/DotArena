#include "circle.hpp"
using namespace std;
Circle::Circle(int id, double r, tuple<double, double, double> pos, tuple<double, double, double> dir) : id(id), radius(r), position(pos), direction(dir) {}
   