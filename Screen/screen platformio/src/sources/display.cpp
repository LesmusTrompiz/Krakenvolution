#include "display.hpp"

// Style
constexpr auto STATISTICS_ICON_COLOR    = display::rgb_to_565(108,203,110);
constexpr auto STRATEGY_ICON_COLOR      = display::rgb_to_565(76,185,214);
constexpr auto DEBUG_ICON_COLOR         = display::rgb_to_565(251, 236, 119);
constexpr auto LIDAR_ICON_COLOR         = display::rgb_to_565(249,45,114);
constexpr auto POWER_ICON_COLOR         = display::WHITE;

constexpr auto GREEN_FIELD_COLOR        = display::rgb_to_565(0, 170, 18);
constexpr auto BLUE_FIELD_COLOR         = display::rgb_to_565(0, 92, 230);

constexpr auto GRAY                     = display::rgb_to_565(64, 64, 64);

namespace display {
    //Segun los atributos un fondo de un icono en blanco (seleccionado) y resto en negro
    void pintarIconos(LCDWIKI_KBV& mylcd, uint8_t selected) {
        //Limpiar zona de escritura
        mylcd.Fill_Rect(  0, 0, 423, 272, BLACK);

        //Limpiar menu secundario
        mylcd.Fill_Rect(    0, 281, 113, 319, BLACK);
        mylcd.Fill_Rect(  121, 281, 113, 319, BLACK);
        mylcd.Fill_Rect(  243, 281, 113, 319, BLACK);
        mylcd.Fill_Rect(  365, 281, 113, 319, BLACK);

        mylcd.Fill_Rect(432,  selected * 56, 48, 49, GRAY);
        //Fondo estadistica
        //Icono estadistica
        mylcd.Fill_Rect(436, 33, 8, 8, STATISTICS_ICON_COLOR);
        mylcd.Fill_Rect(440, 25, 8, 8, STATISTICS_ICON_COLOR);
        mylcd.Fill_Rect(444, 17, 8, 8, STATISTICS_ICON_COLOR);
        mylcd.Fill_Rect(448,  9, 8, 8, STATISTICS_ICON_COLOR);
        mylcd.Fill_Rect(456, 17, 8, 8, STATISTICS_ICON_COLOR);
        mylcd.Fill_Rect(460, 25, 8, 8, STATISTICS_ICON_COLOR);
        mylcd.Fill_Rect(468, 21, 8, 8, STATISTICS_ICON_COLOR);

        //Fondo caballo
        // mylcd.Fill_Rect(432, 57, 48, 48, caballo_c1);
        //Icono caballo
        mylcd.Fill_Rect(440, 93, 32, 8, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(444, 89, 24, 4, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(448, 85, 24, 4, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(452, 81, 24, 4, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(436, 73, 40, 8, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(440, 69, 12, 4, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(456, 69, 16, 4, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(448, 65, 20, 4, STRATEGY_ICON_COLOR);
        mylcd.Fill_Rect(452, 61,  8, 4, STRATEGY_ICON_COLOR);

        //Fondo bicho
        // mylcd.Fill_Rect(432, 113, 48, 48, bicho_c1);
        //Icono bicho
        mylcd.Fill_Rect(444, 153,  4, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(452, 153, 12, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(444, 149, 28, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(436, 145,  4, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(444, 145,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(464, 145,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(436, 141, 12, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(460, 141,  4, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(468, 141,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(440, 137,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(456, 137,  4, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(468, 137,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(440, 133,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(452, 133,  4, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(468, 133,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(436, 129, 16, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(464, 129,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(436, 125, 40, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(440, 121, 24, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(444, 117,  8, 4, DEBUG_ICON_COLOR);
        mylcd.Fill_Rect(460, 117,  8, 4, DEBUG_ICON_COLOR);

        //Fondo lidar
        // mylcd.Fill_Rect(432, 169, 48, 48, lidar_c1);
        //Icono lidar
        mylcd.Fill_Rect(436, 205, 40, 8, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(436, 201,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(472, 201,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(436, 197,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(444, 197, 24, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(472, 197,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(436, 189,  8, 8, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(452, 189,  8, 8, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(468, 189,  8, 8, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(436, 185,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(444, 185, 24, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(472, 185,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(440, 181,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(468, 181,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(444, 177,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(464, 177,  4, 4, LIDAR_ICON_COLOR);
        mylcd.Fill_Rect(448, 173, 16, 4, LIDAR_ICON_COLOR);

        //Fondo apagar
        // mylcd.Fill_Rect(432, 225, 48, 48, apagar_c1);
        //Icono apagar
        mylcd.Fill_Rect(444, 265, 24, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(440, 261,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(464, 261,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(436, 257,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(468, 257,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(436, 253,  4, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(472, 253,  4, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(436, 249,  4, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(472, 249,  4, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(436, 245,  4, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(452, 245,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(472, 245,  4, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(436, 241,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(452, 241,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(468, 241,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(440, 237,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(452, 237,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(464, 237,  8, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(444, 233, 24, 4, POWER_ICON_COLOR);
        mylcd.Fill_Rect(452, 229,  8, 4, POWER_ICON_COLOR);
    }

    void marcoMenuPrincipal(LCDWIKI_KBV& mylcd) {
        //Verticales menu principal
        mylcd.Fill_Rect(424,   0, 8, 280, WHITE);
        mylcd.Fill_Rect(113, 273, 8,  56, WHITE);
        mylcd.Fill_Rect(235, 273, 8,  56, WHITE);
        mylcd.Fill_Rect(357, 273, 8,  56, WHITE);

        //Horizontales menu principal
        mylcd.Fill_Rect(424,  49,  56, 8, WHITE);
        mylcd.Fill_Rect(424, 105,  56, 8, WHITE);
        mylcd.Fill_Rect(424, 161,  56, 8, WHITE);
        mylcd.Fill_Rect(424, 217,  56, 8, WHITE);
        mylcd.Fill_Rect(  0, 273, 480, 8, WHITE);
    }

    void pintarCampo(LCDWIKI_KBV& mylcd, boolean campo, int spawn) {
        mylcd.Fill_Rect(  8, 8, 416, 233, BLACK);

        //Marco del campo
        mylcd.Set_Draw_color(WHITE);
        mylcd.Draw_Rectangle(8, 8, 416, 120);
        mylcd.Draw_Rectangle(8, 121, 416, 233);

        if(campo) {
            mylcd.Set_Draw_color(GREEN_FIELD_COLOR);
            mylcd.Fill_Rectangle(9, 9, 415, 119);
        } else {
            mylcd.Set_Draw_color(GREEN_FIELD_COLOR);
            mylcd.Fill_Rectangle(9, 122, 415, 232);
        }
        mylcd.Set_Draw_color(WHITE);

        //Marcos de los spawns
        mylcd.Draw_Rectangle(  8,   8,  55,  55);
        mylcd.Draw_Rectangle(128,   8, 183,  55);
        mylcd.Draw_Rectangle(248,   8, 303,  55);
        mylcd.Draw_Rectangle(369,   8, 416,  55);
        mylcd.Draw_Rectangle(369,  65, 416, 112);
        mylcd.Draw_Rectangle(369, 129, 416, 177);
        mylcd.Draw_Rectangle(  8, 186,  55, 233);
        mylcd.Draw_Rectangle(128, 186, 183, 233);
        mylcd.Draw_Rectangle(248, 186, 303, 233);
        mylcd.Draw_Rectangle(369, 186, 416, 233);

        mylcd.Draw_Circle(215, 120, 30);

        switch(spawn) {
            case 1:
                mylcd.Fill_Rectangle(  8,   8,  55,  55);
                break;

            case 2:
                mylcd.Fill_Rectangle(128,   8, 183,  55);
                break;

            case 3:
                mylcd.Fill_Rectangle(248,   8, 303,  55);
                break;

            case 4:
                mylcd.Fill_Rectangle(369,   8, 416,  55);
                break;

            case 5:
                mylcd.Fill_Rectangle(369,  65, 416, 112);
                break;

            case 6:
                mylcd.Fill_Rectangle(369, 129, 416, 177);
                break;

            case 7:
                mylcd.Fill_Rectangle(369, 186, 416, 233);
                break;

            case 8:
                mylcd.Fill_Rectangle(248, 186, 303, 233);
                break;

            case 9:
                mylcd.Fill_Rectangle(128, 186, 183, 233);
                break;

            case 10:
                mylcd.Fill_Rectangle(  8, 186,  55, 233);
                break;
        }
    }

    void pintarSpawn(LCDWIKI_KBV& mylcd, boolean campo, int spawn) {
        mylcd.Set_Draw_color(WHITE);

        switch(spawn) {
            case 1:
                mylcd.Fill_Rectangle(  8,   8,  55,  55);

                if(campo) {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                    mylcd.Fill_Rectangle(129, 187, 182, 232);
                } else {
                    mylcd.Set_Draw_color(BLUE_FIELD_COLOR);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                    mylcd.Fill_Rectangle(129, 187, 182, 232);
                }

                break;

            case 2:
                mylcd.Fill_Rectangle(128,   8, 183,  55);

                if(campo) {
                    mylcd.Set_Draw_color(BLUE_FIELD_COLOR);
                    mylcd.Fill_Rectangle(  9,   9,  54,  54);
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                } else {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(  9,   9,  54,  54);
                    mylcd.Set_Draw_color(BLUE_FIELD_COLOR);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                }

                break;

            case 3:
                mylcd.Fill_Rectangle(248,   8, 303,  55);

                if(campo) {
                    mylcd.Set_Draw_color(GREEN_FIELD_COLOR);
                    mylcd.Fill_Rectangle(129,   9, 182,  54);
                    mylcd.Fill_Rectangle(  9,   9,  54,  54);
                } else {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(129,   9, 182,  54);
                    mylcd.Fill_Rectangle(  9,   9,  54,  54);
                }

                break;

            case 4:
                mylcd.Fill_Rectangle(369,   8, 416,  55);

                if(campo) {
                    mylcd.Set_Draw_color(GREEN_FIELD_COLOR);
                    mylcd.Fill_Rectangle(249,   9, 302,  54);
                    mylcd.Fill_Rectangle(129,   9, 182,  54);
                } else {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(249,   9, 302,  54);
                    mylcd.Fill_Rectangle(129,   9, 182,  54);
                }

                break;

            case 5:
                mylcd.Fill_Rectangle(369,  65, 416, 112);

                if(campo) {
                    mylcd.Set_Draw_color(GREEN_FIELD_COLOR);
                    mylcd.Fill_Rectangle(370,   9, 415,  54);
                    mylcd.Fill_Rectangle(249,   9, 302,  54);
                } else {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(370,   9, 415,  54);
                    mylcd.Fill_Rectangle(249,   9, 302,  54);
                }

                break;

            case 6:
                mylcd.Fill_Rectangle(369, 129, 416, 177);

                if(campo) {
                    mylcd.Set_Draw_color(GREEN_FIELD_COLOR);
                    mylcd.Fill_Rectangle(370,  66, 415, 111);
                    mylcd.Fill_Rectangle(370,   9, 415,  54);
                } else {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(370,  66, 415, 111);
                    mylcd.Fill_Rectangle(370,   9, 415,  54);
                }

                break;

            case 7:
                mylcd.Fill_Rectangle(369, 186, 416, 233);

                if(campo) {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(370, 130, 415, 176);
                    mylcd.Set_Draw_color(GREEN_FIELD_COLOR);
                    mylcd.Fill_Rectangle(370,  66, 415, 111);
                } else {
                    mylcd.Set_Draw_color(BLUE_FIELD_COLOR);
                    mylcd.Fill_Rectangle(370, 130, 415, 176);
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(370,  66, 415, 111);
                }

                break;

            case 8:
                mylcd.Fill_Rectangle(248, 186, 303, 233);

                if(campo) {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(370, 187, 415, 232);
                    mylcd.Fill_Rectangle(370, 130, 415, 176);
                } else {
                    mylcd.Set_Draw_color(BLUE_FIELD_COLOR);
                    mylcd.Fill_Rectangle(370, 187, 415, 232);
                    mylcd.Fill_Rectangle(370, 130, 415, 176);
                }

                break;

            case 9:
                mylcd.Fill_Rectangle(128, 186, 183, 233);

                if(campo) {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(249, 187, 302, 232);
                    mylcd.Fill_Rectangle(370, 187, 415, 232);
                } else {
                    mylcd.Set_Draw_color(BLUE_FIELD_COLOR);
                    mylcd.Fill_Rectangle(249, 187, 302, 232);
                    mylcd.Fill_Rectangle(370, 187, 415, 232);
                }

                break;

            case 10:
                mylcd.Fill_Rectangle(  8, 186,  55, 233);

                if(campo) {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(129, 187, 182, 232);
                    mylcd.Fill_Rectangle(249, 187, 302, 232);
                } else {
                    mylcd.Set_Draw_color(BLUE_FIELD_COLOR);
                    mylcd.Fill_Rectangle(129, 187, 182, 232);
                    mylcd.Fill_Rectangle(249, 187, 302, 232);
                }

                break;
        }
    }

    void escribirTexto(LCDWIKI_KBV &mylcd, uint16_t color, uint8_t tamanno, String texto, int coordenada_X, int coordenada_Y) {
        mylcd.Set_Text_colour(color);
        mylcd.Set_Text_Size(tamanno);
        mylcd.Print_String(texto, coordenada_X, coordenada_Y);
    }
}

// Segmented text
using namespace display;

SegmentedText::SegmentedText(const char *txt, size_t x, size_t y, size_t width, size_t height) 
    : m_txt(txt), x(x), y(y), width(width), height(height) {
    reset();
}

SegmentedText::SegmentedText(const char* txt, size_t x, size_t y) 
    : m_txt(txt), x(x), y(y), width(-1), height(-1) {
    reset();
}

void SegmentedText::reset() {
    m_index = 0;
    m_writing = true;
    last_x = x;
    last_y = y;
}

void SegmentedText::update(LCDWIKI_KBV& lcd) {
    if (!m_writing)
        return;
    // Get text dimensions
    size_t CHAR_WIDTH = 6 * lcd.Get_Text_Size() - 1;
    size_t CHAR_HEIGHT = 8 * lcd.Get_Text_Size() - 1;
    char CURRENT_CHAR = m_txt[m_index];
    m_index++;

    auto newline = [=, &lcd, this]() {
        last_y += CHAR_HEIGHT;
        last_x = x;
    };

    // Print character
    switch (CURRENT_CHAR) {
        case '\0':
            m_writing = false;
            break;
        case '\n':
            newline();
            break;
        default:
            // Check boundaries
            if ((last_x + CHAR_WIDTH * 2) >= (x + width)) {
                newline();
            } else {
                last_x += CHAR_WIDTH;
            }
            if ((last_y - y) >= (height)) {
                m_writing = false;
                break;
            }

            // Write character to LCD 
            // lcd.Set_Text_Cousur(last_x, last_y);
            auto color = lcd.Get_Text_colour();
            auto bg = lcd.Get_Text_Back_colour();
            auto size = lcd.Get_Text_Size();
            auto mode = lcd.Get_Text_Mode();

            lcd.Draw_Char(last_x, last_y, CURRENT_CHAR, color, bg, size, mode);
    }
}
