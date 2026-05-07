#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <functional>
#include "arena.hpp"
#include "circle.hpp"

using Vector3 = std::tuple<double, double, double>;

class Renderer {
public:
    Renderer(int width = 800, int height = 800);
    
    // Generates the GIF by orchestrating the render loop and invoking the physics step callback
    void renderToGif(const Arena& arena, const std::vector<Circle>& circles, 
                     std::function<void()> step_func, 
                     std::string filename, int frames, double dt);

private:
    int width;
    int height;
    double focal_length;
    Vector3 light_dir;

    void drawLine(std::vector<uint8_t>& image, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b);
    std::pair<int, int> projectPoint(double x, double y, double z, double camera_z);
    void drawBoundingBox(std::vector<uint8_t>& image, const Arena& arena, double camera_z);
    void drawCircles(std::vector<uint8_t>& image, const std::vector<Circle>& circles, double camera_z);
};

#endif
