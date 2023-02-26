#include <LCDWIKI_KBV.h>
#include "../../include/checkButtons.hpp"

namespace checkButtons {
    //Devolvemos el boton pulsado con prioridad de menor a mayor
    int mirarBotonesPrincipal(int menuEstadistica, int menuCaballo,
    int menuBicho, int menuLidar, int menuApagar, int menuActual) {
    if(!digitalRead(menuEstadistica) && menuActual != 1)
        return 1;
    else if(!digitalRead(menuCaballo) && menuActual != 2)
        return 2;
    else if(!digitalRead(menuBicho) && menuActual != 3)
        return 3;
    else if(!digitalRead(menuLidar) && menuActual != 4)
        return 4;
    else if(!digitalRead(menuApagar) && menuActual != 5)
        return 5;
    else
        return 0;
    }

    //Hacemos lo mismo que en el metodo mirarBotonesPrincipal() pero con
    //los botones del menu secundario
    int mirarBotonesSecundario(int secundario_b1, int secundario_b2,
    int secundario_b3, int secundario_b4) {
    if(!digitalRead(secundario_b1))
        return 1;
    else if(!digitalRead(secundario_b2))
        return 2;
    else if(!digitalRead(secundario_b3))
        return 3;
    else if(!digitalRead(secundario_b4))
        return 4;
    else
        return 0;
    }
}