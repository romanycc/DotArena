#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include "arena.hpp"

using Vector3 = std::tuple<double, double, double>;

/**
 * @class Renderer
 * @brief Handles 3D projection rendering and outputs animated simulation GIFs.
 */
class Renderer {
public:
    /**
     * @brief Constructs a new Renderer.
     * @param width Image width in pixels. Default is 800.
     * @param height Image height in pixels. Default is 800.
     */
    Renderer(int width = 800, int height = 800);
    
    /**
     * @brief Steps the simulation and renders each frame to a GIF file.
     * @param sim Reference to the DotArena physics engine.
     * @param step_func Callback function to advance the simulation state by dt.
     * @param filename Path of the output GIF file.
     * @param frames Number of frames to simulate and render.
     * @param dt Step size of the simulation (delta time).
     */
    void renderToGif(const class DotArena& sim, 
                     std::function<void()> step_func, 
                     std::string filename, int frames, double dt);

private:
    int width;            ///< Render canvas width in pixels.
    int height;           ///< Render canvas height in pixels.
    double focal_length;  ///< Focal length used for 3D-to-2D perspective projection.
    Vector3 light_dir;    ///< Normalized light direction vector for diffuse shading.

    /**
     * @brief Draws a colored 2D line on the canvas buffer using Bresenham's algorithm.
     */
    void drawLine(std::vector<uint8_t>& image, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Performs perspective projection mapping a 3D coordinate to 2D canvas coordinates.
     */
    std::pair<int, int> projectPoint(double x, double y, double z, double camera_z);

    /**
     * @brief Draws the 3D bounding box wireframe of the Arena.
     */
    void drawBoundingBox(std::vector<uint8_t>& image, const Arena& arena, double camera_z);

    /**
     * @brief Depth-sorts and renders all active simulation circles with diffuse shading.
     */
    void drawCircles(std::vector<uint8_t>& image, const class DotArena& sim, double camera_z);
};

#endif // RENDERER_HPP
