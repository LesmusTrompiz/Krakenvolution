#include "LCDWIKI_KBV.h"
#include "checkButtons.hpp"

//Pines menu principal
constexpr int menuEstadistica = 43;
constexpr int menuCaballo = 41;
constexpr int menuBicho = 39;
constexpr int menuLidar = 37;
constexpr int menuApagar = 35;

//Pines menu secundario
constexpr int secundario_b1 = 47;
constexpr int secundario_b2 = 49;
constexpr int secundario_b3 = 51;
constexpr int secundario_b4 = 53;

namespace checkButtons {
    //Devolvemos el boton pulsado con prioridad de menor a mayor
    int mirarBotonesPrincipal(int menuActual) {
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

    //Hacemos lo mismo que en el metodo mirarBotonesPrincipal() pero con los botones del menu secundario
    int mirarBotonesSecundario() {
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

    int getMenuEstadistica() {return menuEstadistica;}
    int getMenuCaballo() {return menuCaballo;}
    int getMenuBicho() {return menuBicho;}
    int getMenuLidar() {return menuLidar;}
    int getMenuApagar() {return menuApagar;}
    int getSecundario_b1() {return secundario_b1;}
    int getSecundario_b2() {return secundario_b2;}
    int getSecundario_b3() {return secundario_b3;}
    int getSecundario_b4() {return secundario_b4;}
}
