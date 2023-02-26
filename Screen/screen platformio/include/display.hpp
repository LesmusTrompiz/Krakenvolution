#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <LCDWIKI_KBV.h>

namespace display {
    //Segun los atributos un fondo de un icono en blanco (seleccionado) y resto en negro
    void pintarIconos(LCDWIKI_KBV mylcd, uint16_t estadistica_c1, uint16_t estadistica_c2,
    uint16_t caballo_c1, uint16_t caballo_c2, uint16_t bicho_c1, uint16_t bicho_c2,
    uint16_t lidar_c1, uint16_t lidar_c2, uint16_t apagar_c1, uint16_t apagar_c2);

    void marcoMenuPrincipal(LCDWIKI_KBV mylcd);

    /**
    void pintarIconosConexion();

    void pintarCampo();
    */

    void escribirTexto(LCDWIKI_KBV mylcd, uint16_t color, uint8_t tamanno, String texto,
    uint8_t coordenada_X, uint8_t coordenada_Y);

    //Si no hay interrupciones devuelve 0, si las hay devuelve el codigo del menu que hay que ejecutar
    int escribirErrores(LCDWIKI_KBV mylcd, uint16_t color, uint8_t tamanno, String texto,
    uint8_t coordenada_X, uint8_t coordenada_Y, int menuEstadistica, int menuCaballo,
    int menuBicho, int menuLidar, int menuApagar, int menuActual);
}

#endif