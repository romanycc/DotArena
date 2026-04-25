#include "arena.hpp"
using namespace std;
Arena::Arena(std::tuple<double, double, double> s, double sr, double fc) 
    : size(s), shrink_rate(sr), friction_coefficient(fc) {}
