#include "dotarena.hpp"
#include "gif.h"
#include <cmath>

Renderer::Renderer(int w, int h) 
    : width(w), height(h), focal_length(500.0), light_dir({0.577, -0.577, -0.577}) {}

void Renderer::drawLine(std::vector<uint8_t>& image, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    while (true) {
        if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
            int idx = (y0 * width + x0) * 4;
            image[idx] = r; image[idx + 1] = g; image[idx + 2] = b; image[idx + 3] = 255;
        }
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

std::pair<int, int> Renderer::projectPoint(double x, double y, double z, double camera_z) {
    double z_rel = z + camera_z;
    if (z_rel <= 0.1) z_rel = 0.1;
    double scale = focal_length / z_rel;
    return {(int)(x * scale + width / 2.0), (int)(y * scale + height / 2.0)};
}

void Renderer::drawBoundingBox(std::vector<uint8_t>& image, const Arena& arena, double camera_z) {
    double sx_arena = std::get<0>(arena.getSize()) / 2.0;
    double sy_arena = std::get<1>(arena.getSize()) / 2.0;
    double sz_arena = std::get<2>(arena.getSize()) / 2.0;

    Vector3 c[8] = {
        {-sx_arena, -sy_arena, -sz_arena}, {sx_arena, -sy_arena, -sz_arena},
        {sx_arena, sy_arena, -sz_arena}, {-sx_arena, sy_arena, -sz_arena},
        {-sx_arena, -sy_arena, sz_arena}, {sx_arena, -sy_arena, sz_arena},
        {sx_arena, sy_arena, sz_arena}, {-sx_arena, sy_arena, sz_arena}
    };
    std::pair<int, int> p[8];
    for(int i = 0; i < 8; ++i) 
        p[i] = projectPoint(std::get<0>(c[i]), std::get<1>(c[i]), std::get<2>(c[i]), camera_z);

    uint8_t br = 100, bg = 100, bb = 255; // Blue wireframe
    // Bottom face
    drawLine(image, p[0].first, p[0].second, p[1].first, p[1].second, br, bg, bb);
    drawLine(image, p[1].first, p[1].second, p[2].first, p[2].second, br, bg, bb);
    drawLine(image, p[2].first, p[2].second, p[3].first, p[3].second, br, bg, bb);
    drawLine(image, p[3].first, p[3].second, p[0].first, p[0].second, br, bg, bb);
    // Top face
    drawLine(image, p[4].first, p[4].second, p[5].first, p[5].second, br, bg, bb);
    drawLine(image, p[5].first, p[5].second, p[6].first, p[6].second, br, bg, bb);
    drawLine(image, p[6].first, p[6].second, p[7].first, p[7].second, br, bg, bb);
    drawLine(image, p[7].first, p[7].second, p[4].first, p[4].second, br, bg, bb);
    // Pillars
    drawLine(image, p[0].first, p[0].second, p[4].first, p[4].second, br, bg, bb);
    drawLine(image, p[1].first, p[1].second, p[5].first, p[5].second, br, bg, bb);
    drawLine(image, p[2].first, p[2].second, p[6].first, p[6].second, br, bg, bb);
    drawLine(image, p[3].first, p[3].second, p[7].first, p[7].second, br, bg, bb);
}

void Renderer::drawCircles(std::vector<uint8_t>& image, const DotArena& sim, double camera_z) {
    int num_circles = sim.getNumCircles();
    if (num_circles == 0) return;

    const auto& px = sim.getPx();
    const auto& py = sim.getPy();
    const auto& pz = sim.getPz();
    const auto& radius = sim.getRadius();

    std::vector<int> sorted_indices(num_circles);
    for(int i = 0; i < num_circles; ++i) sorted_indices[i] = i;

    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&pz](int a, int b) {
                return pz[a] > pz[b];
              });

    for (int idx : sorted_indices) {
      double z = pz[idx];
      double z_rel = z + camera_z;
      if (z_rel <= 0.1) continue;
      double scale = focal_length / z_rel;

      double cx = px[idx] * scale + width / 2.0;
      double cy = py[idx] * scale + height / 2.0;
      double r = radius[idx] * scale;

      int minX = std::max(0, (int)(cx - r));
      int maxX = std::min(width - 1, (int)(cx + r));
      int minY = std::max(0, (int)(cy - r));
      int maxY = std::min(height - 1, (int)(cy + r));

      for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
          double dx = x - cx;
          double dy = y - cy;
          double distSq = dx * dx + dy * dy;
          if (distSq <= r * r) {
            double nx = dx / r;
            double ny = dy / r;
            double nz = -std::sqrt(std::max(0.0, 1.0 - nx * nx - ny * ny));

            double diffuse = std::max(0.0, nx * std::get<0>(light_dir) +
                                               ny * std::get<1>(light_dir) +
                                               nz * std::get<2>(light_dir));
            double ambient = 0.3;
            double intensity = std::min(1.0, ambient + diffuse);

            int r_col = std::min(255, (int)(255 * intensity));
            int g_col = std::min(255, (int)(100 * intensity));
            int b_col = std::min(255, (int)(50 * intensity));

            int index = (y * width + x) * 4;
            image[index + 0] = r_col;
            image[index + 1] = g_col;
            image[index + 2] = b_col;
            image[index + 3] = 255;
          }
        }
      }
    }
}

void Renderer::renderToGif(const DotArena& sim, 
                           std::function<void()> step_func, 
                           std::string filename, int frames, double dt) {
    const Arena& arena = sim.getArena();
    int delay = std::max(2, (int)(dt * 100)); // delay in hundredths of a second
    GifWriter g;
    GifBegin(&g, filename.c_str(), width, height, delay);
    
    double sx_arena = std::get<0>(arena.getSize()) / 2.0;
    double sy_arena = std::get<1>(arena.getSize()) / 2.0;
    double sz_arena = std::get<2>(arena.getSize()) / 2.0;
    double max_dim = std::max({sx_arena, sy_arena, sz_arena});
    double camera_z = max_dim * 2.5; // Push camera back

    for (int f = 0; f < frames; ++f) {
        std::vector<uint8_t> image(width * height * 4, 0); // black background

        drawBoundingBox(image, arena, camera_z);
        drawCircles(image, sim, camera_z);

        GifWriteFrame(&g, image.data(), width, height, delay);
        step_func(); // Advance the simulation!
    }
    GifEnd(&g);
}
