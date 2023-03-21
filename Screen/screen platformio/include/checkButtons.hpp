#ifndef CHECKBUTTONS_HPP
#define CHECKBUTTONS_HPP

#include "LCDWIKI_KBV.h"

namespace checkButtons {
    //Devolvemos el boton pulsado con prioridad de menor a mayor
    int mirarBotonesPrincipal(int menuActual);
    //Hacemos lo mismo que en el metodo mirarBotonesPrincipal() pero con los botones del menu secundario
    int mirarBotonesSecundario(int secundario_b1, int secundario_b2,
    int secundario_b3, int secundario_b4);
}

#endif
