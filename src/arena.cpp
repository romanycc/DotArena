#include "arena.hpp"
using namespace std;
Arena::Arena(std::tuple<double, double, double> s, double fc) 
    : size(s), friction_coefficient(fc) {}
