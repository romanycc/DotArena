#ifndef ARENA_HPP
#define ARENA_HPP
#include <tuple>
using namespace std;
class Arena{
    public:
        Arena(tuple<double, double, double> size, double shrink_rate, double friction_coefficient);
    private:
        tuple<double, double, double> size;
        double shrink_rate;
        double friction_coefficient;
};
#endif