#include "LCDWIKI_KBV.h"
#include "LCDWIKI_GUI.h"
#include "display.hpp"
#include "checkButtons.hpp"

//Colores
#define BLACK       0x0000
#define DARKBLUE    0x000F
#define DARKGREEN   0x00E0
#define WHITE       0xFFFF

namespace display {
    //Segun los atributos un fondo de un icono en blanco (seleccionado) y resto en negro
    void pintarIconos(LCDWIKI_KBV mylcd, uint16_t estadistica_c1, uint16_t estadistica_c2,
    uint16_t caballo_c1, uint16_t caballo_c2, uint16_t bicho_c1, uint16_t bicho_c2,
    uint16_t lidar_c1, uint16_t lidar_c2, uint16_t apagar_c1, uint16_t apagar_c2) {
        //Limpiar zona de escritura
        mylcd.Fill_Rect(  0, 0, 423, 272, BLACK);

        //Limpiar menu secundario
        mylcd.Fill_Rect(    0, 281, 113, 319, BLACK);
        mylcd.Fill_Rect(  121, 281, 113, 319, BLACK);
        mylcd.Fill_Rect(  243, 281, 113, 319, BLACK);
        mylcd.Fill_Rect(  365, 281, 113, 319, BLACK);

        //Fondo estadistica
        mylcd.Fill_Rect(432, 0, 48, 49, estadistica_c1);
        //Icono estadistica
        mylcd.Fill_Rect(436, 33, 8, 8, estadistica_c2);
        mylcd.Fill_Rect(440, 25, 8, 8, estadistica_c2);
        mylcd.Fill_Rect(444, 17, 8, 8, estadistica_c2);
        mylcd.Fill_Rect(448,  9, 8, 8, estadistica_c2);
        mylcd.Fill_Rect(456, 17, 8, 8, estadistica_c2);
        mylcd.Fill_Rect(460, 25, 8, 8, estadistica_c2);
        mylcd.Fill_Rect(468, 21, 8, 8, estadistica_c2);

        //Fondo caballo
        mylcd.Fill_Rect(432, 57, 48, 48, caballo_c1);
        //Icono caballo
        mylcd.Fill_Rect(440, 93, 32, 8, caballo_c2);
        mylcd.Fill_Rect(444, 89, 24, 4, caballo_c2);
        mylcd.Fill_Rect(448, 85, 24, 4, caballo_c2);
        mylcd.Fill_Rect(452, 81, 24, 4, caballo_c2);
        mylcd.Fill_Rect(436, 73, 40, 8, caballo_c2);
        mylcd.Fill_Rect(440, 69, 12, 4, caballo_c2);
        mylcd.Fill_Rect(456, 69, 16, 4, caballo_c2);
        mylcd.Fill_Rect(448, 65, 20, 4, caballo_c2);
        mylcd.Fill_Rect(452, 61,  8, 4, caballo_c2);

        //Fondo bicho
        mylcd.Fill_Rect(432, 113, 48, 48, bicho_c1);
        //Icono bicho
        mylcd.Fill_Rect(444, 153,  4, 4, bicho_c2);
        mylcd.Fill_Rect(452, 153, 12, 4, bicho_c2);
        mylcd.Fill_Rect(444, 149, 28, 4, bicho_c2);
        mylcd.Fill_Rect(436, 145,  4, 4, bicho_c2);
        mylcd.Fill_Rect(444, 145,  8, 4, bicho_c2);
        mylcd.Fill_Rect(464, 145,  8, 4, bicho_c2);
        mylcd.Fill_Rect(436, 141, 12, 4, bicho_c2);
        mylcd.Fill_Rect(460, 141,  4, 4, bicho_c2);
        mylcd.Fill_Rect(468, 141,  8, 4, bicho_c2);
        mylcd.Fill_Rect(440, 137,  8, 4, bicho_c2);
        mylcd.Fill_Rect(456, 137,  4, 4, bicho_c2);
        mylcd.Fill_Rect(468, 137,  8, 4, bicho_c2);
        mylcd.Fill_Rect(440, 133,  8, 4, bicho_c2);
        mylcd.Fill_Rect(452, 133,  4, 4, bicho_c2);
        mylcd.Fill_Rect(468, 133,  8, 4, bicho_c2);
        mylcd.Fill_Rect(436, 129, 16, 4, bicho_c2);
        mylcd.Fill_Rect(464, 129,  8, 4, bicho_c2);
        mylcd.Fill_Rect(436, 125, 40, 4, bicho_c2);
        mylcd.Fill_Rect(440, 121, 24, 4, bicho_c2);
        mylcd.Fill_Rect(444, 117,  8, 4, bicho_c2);
        mylcd.Fill_Rect(460, 117,  8, 4, bicho_c2);

        //Fondo lidar
        mylcd.Fill_Rect(432, 169, 48, 48, lidar_c1);
        //Icono lidar
        mylcd.Fill_Rect(436, 205, 40, 8, lidar_c2);
        mylcd.Fill_Rect(436, 201,  4, 4, lidar_c2);
        mylcd.Fill_Rect(472, 201,  4, 4, lidar_c2);
        mylcd.Fill_Rect(436, 197,  4, 4, lidar_c2);
        mylcd.Fill_Rect(444, 197, 24, 4, lidar_c2);
        mylcd.Fill_Rect(472, 197,  4, 4, lidar_c2);
        mylcd.Fill_Rect(436, 189,  8, 8, lidar_c2);
        mylcd.Fill_Rect(452, 189,  8, 8, lidar_c2);
        mylcd.Fill_Rect(468, 189,  8, 8, lidar_c2);
        mylcd.Fill_Rect(436, 185,  4, 4, lidar_c2);
        mylcd.Fill_Rect(444, 185, 24, 4, lidar_c2);
        mylcd.Fill_Rect(472, 185,  4, 4, lidar_c2);
        mylcd.Fill_Rect(440, 181,  4, 4, lidar_c2);
        mylcd.Fill_Rect(468, 181,  4, 4, lidar_c2);
        mylcd.Fill_Rect(444, 177,  4, 4, lidar_c2);
        mylcd.Fill_Rect(464, 177,  4, 4, lidar_c2);
        mylcd.Fill_Rect(448, 173, 16, 4, lidar_c2);

        //Fondo apagar
        mylcd.Fill_Rect(432, 225, 48, 48, apagar_c1);
        //Icono apagar
        mylcd.Fill_Rect(444, 265, 24, 4, apagar_c2);
        mylcd.Fill_Rect(440, 261,  8, 4, apagar_c2);
        mylcd.Fill_Rect(464, 261,  8, 4, apagar_c2);
        mylcd.Fill_Rect(436, 257,  8, 4, apagar_c2);
        mylcd.Fill_Rect(468, 257,  8, 4, apagar_c2);
        mylcd.Fill_Rect(436, 253,  4, 4, apagar_c2);
        mylcd.Fill_Rect(472, 253,  4, 4, apagar_c2);
        mylcd.Fill_Rect(436, 249,  4, 4, apagar_c2);
        mylcd.Fill_Rect(472, 249,  4, 4, apagar_c2);
        mylcd.Fill_Rect(436, 245,  4, 4, apagar_c2);
        mylcd.Fill_Rect(452, 245,  8, 4, apagar_c2);
        mylcd.Fill_Rect(472, 245,  4, 4, apagar_c2);
        mylcd.Fill_Rect(436, 241,  8, 4, apagar_c2);
        mylcd.Fill_Rect(452, 241,  8, 4, apagar_c2);
        mylcd.Fill_Rect(468, 241,  8, 4, apagar_c2);
        mylcd.Fill_Rect(440, 237,  8, 4, apagar_c2);
        mylcd.Fill_Rect(452, 237,  8, 4, apagar_c2);
        mylcd.Fill_Rect(464, 237,  8, 4, apagar_c2);
        mylcd.Fill_Rect(444, 233, 24, 4, apagar_c2);
        mylcd.Fill_Rect(452, 229,  8, 4, apagar_c2);
    }

    void marcoMenuPrincipal(LCDWIKI_KBV mylcd) {
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

    void pintarCampo(LCDWIKI_KBV mylcd, boolean campo, int spawn) {
        mylcd.Fill_Rect(  8, 8, 416, 233, BLACK);

        //Marco del campo
        mylcd.Set_Draw_color(WHITE);
        mylcd.Draw_Rectangle(8, 8, 416, 120);
        mylcd.Draw_Rectangle(8, 121, 416, 233);

        if(campo) {
            mylcd.Set_Draw_color(DARKGREEN);
            mylcd.Fill_Rectangle(9, 9, 415, 119);
        } else {
            mylcd.Set_Draw_color(DARKBLUE);
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

    void pintarSpawn(LCDWIKI_KBV mylcd, boolean campo, int spawn) {
        mylcd.Set_Draw_color(WHITE);

        switch(spawn) {
            case 1:
                mylcd.Fill_Rectangle(  8,   8,  55,  55);

                if(campo) {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                    mylcd.Fill_Rectangle(129, 187, 182, 232);
                } else {
                    mylcd.Set_Draw_color(DARKBLUE);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                    mylcd.Fill_Rectangle(129, 187, 182, 232);
                }

                break;

            case 2:
                mylcd.Fill_Rectangle(128,   8, 183,  55);

                if(campo) {
                    mylcd.Set_Draw_color(DARKGREEN);
                    mylcd.Fill_Rectangle(  9,   9,  54,  54);
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                } else {
                    mylcd.Set_Draw_color(BLACK);
                    mylcd.Fill_Rectangle(  9,   9,  54,  54);
                    mylcd.Set_Draw_color(DARKBLUE);
                    mylcd.Fill_Rectangle(  9, 187,  54, 232);
                }

                break;

            case 3:
                mylcd.Fill_Rectangle(248,   8, 303,  55);

                if(campo) {
                    mylcd.Set_Draw_color(DARKGREEN);
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
                    mylcd.Set_Draw_color(DARKGREEN);
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
                    mylcd.Set_Draw_color(DARKGREEN);
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
                    mylcd.Set_Draw_color(DARKGREEN);
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
                    mylcd.Set_Draw_color(DARKGREEN);
                    mylcd.Fill_Rectangle(370,  66, 415, 111);
                } else {
                    mylcd.Set_Draw_color(DARKBLUE);
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
                    mylcd.Set_Draw_color(DARKBLUE);
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
                    mylcd.Set_Draw_color(DARKBLUE);
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
                    mylcd.Set_Draw_color(DARKBLUE);
                    mylcd.Fill_Rectangle(129, 187, 182, 232);
                    mylcd.Fill_Rectangle(249, 187, 302, 232);
                }

                break;
        }
    }

    void pintarPlan(LCDWIKI_KBV mylcd, int plan) {
        mylcd.Set_Draw_color(BLACK);
        mylcd.Fill_Rectangle(8, 241, 420, 265);
        mylcd.Set_Draw_color(WHITE);

        switch(plan) {
            case 1:
                escribirTexto(mylcd, WHITE, 3, "Plan agresivo", 8, 241);
                break;
            case 2:
                escribirTexto(mylcd, WHITE, 3, "Plan defensivo", 8, 241);
                break;
            case 3:
                escribirTexto(mylcd, WHITE, 3, "Plan a medio campo", 8, 241);
                break;
            case 4:
                escribirTexto(mylcd, WHITE, 3, "Plan cuarto de campo", 8, 241);
                break;
            case 5:
                escribirTexto(mylcd, WHITE, 3, "Explotar robot", 8, 241);
                break;
        }
    }
    
    void escribirTexto(LCDWIKI_KBV mylcd, uint16_t color, uint8_t tamanno, String texto,
    int coordenada_X, int coordenada_Y) {
        mylcd.Set_Text_colour(color);
        mylcd.Set_Text_Size(tamanno);
        mylcd.Print_String(texto, coordenada_X, coordenada_Y);
    }

    void ordenarErrores(String errores[13], String error) {
        String linea;
        int longitudLinea = 32;

        while(error.length() > 0) {
            linea = "";

            //Ultima linea
            if(error.length() < longitudLinea) {
                linea += error.substring(0, error.length());
                error.remove(0, error.length());
            //Resto de lineas
            } else {
                //Mientras la suma de la longitud de la varible linea mas la longitud de la seccion del texto
                //hasta el primer espacio en blanco sea menor que la linea maxima permitida hacer
                while(linea.length() + error.indexOf(" ") < longitudLinea && !(error.indexOf(" ") == -1)) {
                    //Insertar la seccion del texto hasta el primer espacio en blanco en linea y borrarla de texto
                    linea += error.substring(0, error.indexOf(" ") + 1);
                    error.remove(0, error.indexOf(" ") + 1);
                }
            }

            //Recolocación de las lineas para insertar nuevas, todas se desplazan una posicion
            //hacia la izquierda en el array, si el array no esta lleno tan solo se insertan
            for(int i = 0; i < 13; i++) {
                if(errores[i] == "") {
                    errores[i] = linea;
                    i = 13;
                } else if(i == 12) {
                    for(int j = 0; j < 13; j++) {
                        if(j != 12)
                            errores[j] = errores[j + 1];
                        else
                            errores[j] = linea;
                    }
                }
            }
        }
    }

    //Si no hay interrupciones devuelve 0, si las hay devuelve el codigo del menu que hay que ejecutar
    int escribirErrores(LCDWIKI_KBV mylcd, uint16_t color, uint8_t tamanno, String errores[13],
    uint8_t coordenada_X, uint8_t coordenada_Y, int menuActual, int secundario_b1) {
        boolean pausa = false;
        boolean estadoSecundario_1 = false;
        //Variables para el parrafo
        int interlineado = 20;
        int menuDevuelto;
        mylcd.Set_Text_colour(color);
        mylcd.Set_Text_Size(tamanno);

        //Limpiar zona de escritura
        mylcd.Fill_Rect(  0, 0, 423, 272, BLACK);

        for(int i = 0; i < 13; i++) {
            //Ventana de medio segundo antes de pintar cada linea para comprobar botones principales
            for(uint32_t inicio = millis(); millis() - inicio < 500;) {
                if((menuDevuelto = checkButtons::mirarBotonesPrincipal(menuActual)) != 0)
                    return menuDevuelto;

                if(digitalRead(secundario_b1))
                    estadoSecundario_1 = false;

                //Control de las dobles pulsaciones
                if(!digitalRead(secundario_b1) && !pausa && !estadoSecundario_1) {
                    pausa = true;
                    estadoSecundario_1 = true;
                } else if(!digitalRead(secundario_b1) && pausa && !estadoSecundario_1) {
                    pausa = false;
                    estadoSecundario_1 = true;
                }

                //Control de la pausa
                if(pausa)
                    inicio = millis();
            }
            mylcd.Print_String(errores[i], coordenada_X, coordenada_Y + i * interlineado);
        }
        return 0;
    }
}
