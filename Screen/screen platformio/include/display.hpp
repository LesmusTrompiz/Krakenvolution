#pragma once

#include "LCDWIKI_GUI.h"
#include "LCDWIKI_KBV.h"

#include <stdint.h>

/// Tamaño de la fuente
///((x + 6 * size - 1) < 0) || ((y + 8 * size - 1)
///

namespace display
{
// Segun los atributos un fondo de un icono en blanco (seleccionado) y resto en negro
void pintarIconos(LCDWIKI_KBV &mylcd, uint8_t selected);

void marcoMenuPrincipal(LCDWIKI_KBV &mylcd);

void pintarCampo(LCDWIKI_KBV &mylcd, boolean campo, int spawn);

void pintarSpawn(LCDWIKI_KBV &mylcd, boolean campo, int spawn);

void pintarPlan(LCDWIKI_KBV &mylcd, int plan);

void escribirTexto(LCDWIKI_KBV &mylcd, uint16_t color, uint8_t tamanno, String texto, int coordenada_X, int coordenada_Y);

constexpr uint16_t rgb_to_565(const uint8_t r, const uint8_t g, const uint8_t b)
{
   constexpr uint16_t POWER5 = 31;
   constexpr uint16_t POWER6 = 63;

   uint16_t color = 0;

   color |= ((r * POWER5) / 255) << (16 - 5);
   color |= ((g * POWER6) / 255) << (16 - 5 - 6);
   color |= ((b * POWER5) / 255);

   return color;
}

class SegmentedText
{
 public:
    SegmentedText(const char* txt, size_t x, size_t y);
    SegmentedText(const char *txt, size_t x, size_t y, size_t width, size_t height);

    void reset();
    void update(LCDWIKI_KBV& lcd);

 private:
    const char *m_txt;
    size_t m_index;
    size_t x, y, width, height, last_x, last_y;
    bool m_writing = true;
};

// constexpr uint16_t BLACK = display::rgb_to_565(27, 32, 38);
constexpr uint16_t BLACK = display::rgb_to_565(0, 0, 0);
constexpr uint16_t WHITE = display::rgb_to_565(255, 255, 255);

constexpr bool IGNORE_BG = true;
constexpr bool DRAW_BG = false;
} // namespace display
