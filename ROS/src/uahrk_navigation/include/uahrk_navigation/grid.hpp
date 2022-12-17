#pragma once

template<unsigned int w, unsigned h>
struct Grid{
    unsigned int width  = w;
    unsigned int height = h;
    unsigned int cells[w][h] {0};
};
