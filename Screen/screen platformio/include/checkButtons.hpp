#ifndef CHECKBUTTONS_HPP
#define CHECKBUTTONS_HPP

namespace checkButtons {
    //Devolvemos el boton pulsado con prioridad de menor a mayor
    int mirarBotonesPrincipal(int menuActual);
    //Hacemos lo mismo que en el metodo mirarBotonesPrincipal() pero con los botones del menu secundario
    int mirarBotonesSecundario();
    int mirarLidar(LCDWIKI_KBV mylcd, int menuActual, uint16_t BLACK);
    int getMenuEstadistica();
    int getMenuCaballo();
    int getMenuBicho();
    int getMenuLidar();
    int getMenuApagar();
    int getSecundario_b1();
    int getSecundario_b2();
    int getSecundario_b3();
    int getSecundario_b4();
    boolean getEstadoSecundario_1();
    boolean getEstadoSecundario_2();
    boolean getEstadoSecundario_3();
    boolean getEstadoSecundario_4();
    void setEstadoSecundario_1(boolean b);
    void setEstadoSecundario_2(boolean b);
    void setEstadoSecundario_3(boolean b);
    void setEstadoSecundario_4(boolean b);
}

#endif
