#ifndef CIRCLE_HPP
#define CIRCLE_HPP
#include <tuple>
using namespace std;
class Circle {
    public:
        Circle(int id, double r, tuple<double, double, double> pos, tuple<double, double, double> dir);
    private:
        int id;
        double radius;
        tuple<double, double, double> position;
        tuple<double, double, double> direction;
};
#endif
