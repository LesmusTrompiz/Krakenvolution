#ifndef CHECKBUTTONS_HPP
#define CHECKBUTTONS_HPP

#include "LCDWIKI_KBV.h"

namespace checkButtons {
    //Devolvemos el boton pulsado con prioridad de menor a mayor
    int mirarBotonesPrincipal(int menuActual);
    //Hacemos lo mismo que en el metodo mirarBotonesPrincipal() pero con los botones del menu secundario
    int mirarBotonesSecundario();
    int getMenuEstadistica();
    int getMenuCaballo();
    int getMenuBicho();
    int getMenuLidar();
    int getMenuApagar();
    int getSecundario_b1();
    int getSecundario_b2();
    int getSecundario_b3();
    int getSecundario_b4();
}

#endif
