#pragma once



constexpr int   EurobotGridWidth       = 20; // 20 dm
constexpr int   EurobotGridHeight      = 30; // 30 dm
constexpr int   PLAYGROUND_WIDTH       = 2;  // 2m
constexpr int   PLAYGROUND_HEIGHT      = 3;  // 2m

constexpr float EurobotGridResolution  = (float)PLAYGROUND_WIDTH /EurobotGridWidth; // 0.1 meters / cell



template<unsigned int w, unsigned h>
struct Grid{
    unsigned int width  = w;
    unsigned int height = h;
    unsigned int cells[w*h] {100,100,100};
};

typedef Grid<EurobotGridWidth,EurobotGridHeight> eurobot_grid;