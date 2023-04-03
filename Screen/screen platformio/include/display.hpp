#ifndef DISPLAY_HPP
#define DISPLAY_HPP

namespace display {
    //Segun los atributos un fondo de un icono en blanco (seleccionado) y resto en negro
    void pintarIconos(LCDWIKI_KBV mylcd, uint16_t estadistica_c1, uint16_t estadistica_c2,
    uint16_t caballo_c1, uint16_t caballo_c2, uint16_t bicho_c1, uint16_t bicho_c2,
    uint16_t lidar_c1, uint16_t lidar_c2, uint16_t apagar_c1, uint16_t apagar_c2);
    void marcoMenuPrincipal(LCDWIKI_KBV mylcd);
    void pintarCampo(LCDWIKI_KBV mylcd, boolean campo, int spawn);
    void pintarSpawn(LCDWIKI_KBV mylcd, boolean campo, int spawn);
    void pintarPlan(LCDWIKI_KBV mylcd, int plan);
    void escribirTexto(LCDWIKI_KBV mylcd, uint16_t color, uint8_t tamanno, String texto,
    int coordenada_X, int coordenada_Y);
    void ordenarErrores(String errores[13], String error);
    //Si no hay interrupciones devuelve 0, si las hay devuelve el codigo del menu que hay que ejecutar
    int escribirErrores(LCDWIKI_KBV mylcd, uint16_t color, uint8_t tamanno, String errores[13],
    uint8_t coordenada_X, uint8_t coordenada_Y, int menuActual, int secundario_b1);
}

#endif
