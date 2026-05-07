#ifndef ARENA_HPP
#define ARENA_HPP
#include <tuple>
using namespace std;
class Arena{
    public:
        Arena(tuple<double, double, double> size, double friction_coefficient);
        double getFriction() const { return friction_coefficient; }
        tuple<double, double, double> getSize() const { return size; }
    private:
        tuple<double, double, double> size;
        double friction_coefficient;
};
#endif