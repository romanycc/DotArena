#ifndef ARENA_HPP
#define ARENA_HPP

#include <tuple>

using namespace std;

/**
 * @class Arena
 * @brief Represents the physical simulation environment boundaries and friction parameters.
 */
class Arena {
public:
    /**
     * @brief Constructs a new Arena.
     * @param size A 3D tuple (width, height, depth) representing the dimensions of the arena.
     * @param friction_coefficient Friction coefficient applied to particles within the arena.
     */
    Arena(tuple<double, double, double> size, double friction_coefficient);

    /**
     * @brief Gets the friction coefficient of the arena environment.
     * @return Double representing the friction coefficient.
     */
    double getFriction() const { return friction_coefficient; }

    /**
     * @brief Gets the 3D dimensions of the arena.
     * @return Tuple containing (width, height, depth).
     */
    tuple<double, double, double> getSize() const { return size; }

private:
    tuple<double, double, double> size; ///< 3D boundaries bounding box size.
    double friction_coefficient;        ///< Friction decay constant.
};

#endif // ARENA_HPP