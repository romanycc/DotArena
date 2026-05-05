#ifndef CIRCLE_HPP
#define CIRCLE_HPP
#include <tuple>
using namespace std;
class Circle {
public:
  Circle(int id, double r, tuple<double, double, double> pos,
         tuple<double, double, double> dir);
  int getId() { return id; }
  double getRadius() { return radius; }
  tuple<double, double, double> getPos() { return position; }
  tuple<double, double, double> getDir() { return direction; }
  void setPos(const tuple<double, double, double> &pos) { position = pos; }
  void setDir(const tuple<double, double, double> &dir) { direction = dir; }

private:
  int id;
  double radius;
  tuple<double, double, double> position;
  tuple<double, double, double> direction;
};
#endif
