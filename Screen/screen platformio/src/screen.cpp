#include "LCDWIKI_KBV.h"
#include "LCDWIKI_GUI.h"
#include "display.hpp"
#include "checkButtons.hpp"

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV mylcd(ILI9488, A3, A2, A1, A0, A4); //model,cs,cd,wr,rd,reset
//if the IC model is not known and the modules is readable,you can use this constructed function
//LCDWIKI_KBV mylcd(320,480,A3,A2,A1,A0,A4);//width,height,cs,cd,wr,rd,reset

//Colores
#define BLACK       0x0000
#define BLUE        0x007E
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF

//Pines menu principal
constexpr int menuEstadistica = 43;
constexpr int menuCaballo = 41;
constexpr int menuBicho = 39;
constexpr int menuLidar = 37;
constexpr int menuApagar = 35;
int menuActual = 1;

//Pines menu secundario
constexpr int secundario_b1 = 47;
constexpr int secundario_b2 = 49;
constexpr int secundario_b3 = 51;
constexpr int secundario_b4 = 53;

//Estado botones secundarios
boolean estadoSecundario_1 = true;
boolean estadoSecundario_2 = true;
boolean estadoSecundario_3 = true;
boolean estadoSecundario_4 = true;

//Variables para las interrupciones
int menuDevuelto;
int codigoInterrupcion = 0;

boolean campo = true;
int spawn = 1;
int plan = 1;
constexpr int numeroPlanes = 5;
boolean lidar = true;
String errores[13];
boolean nuevaLinea = false;
int lineasPintadas = 0;

void insertarError(String error) {
  display::ordenarErrores(errores, error);
  nuevaLinea = true;
}

int mirarLidar() {
  if(digitalRead(secundario_b1) != estadoSecundario_1 && digitalRead(secundario_b1) == LOW) {
    mylcd.Fill_Rect(  0, 0, 423, 272, BLACK);
    mylcd.Fill_Rect(  0, 281, 113, 319, BLACK);

    if(lidar)
      Serial.write("Apagar lidar");
    else
      Serial.write("Encender lidar");

    estadoSecundario_1 = !estadoSecundario_1;
    lidar = !lidar;

    return 4;
  } else if((menuDevuelto = checkButtons::mirarBotonesPrincipal(menuEstadistica, menuCaballo,
  menuBicho, menuLidar, menuApagar, menuActual)) != 0)
    return menuDevuelto;

  return 0;
}

int ejecutarMenuEstadistica() {
  int devolver;
  display::escribirTexto(mylcd, WHITE, 5, "Tiempo: 62s", 8, 8);
  display::escribirTexto(mylcd, WHITE, 5, "Tartas: 4", 8, 55);

  if((devolver = checkButtons::mirarBotonesPrincipal(menuEstadistica,
  menuCaballo, menuBicho, menuLidar, menuApagar, menuActual)) != 0)
    return devolver;

  display::escribirTexto(mylcd, WHITE, 5, "Puntos: 32", 8, 102);
  display::escribirTexto(mylcd, MAGENTA, 6, "UAHRKrakens", 8, 223);

  return 0;
}

void ejecutarMenuCaballo() {
  int devolver;

  switch(devolver = checkButtons::mirarBotonesSecundario(secundario_b1, secundario_b2, secundario_b3, secundario_b4)) {
    case 1:
      if(digitalRead(secundario_b1) != estadoSecundario_1) {
        Serial.write("Robot listo para jugar");
        estadoSecundario_1 = !estadoSecundario_1;
      }
      break;

    case 2:
      if(digitalRead(secundario_b2) != estadoSecundario_2) {
        campo = !campo;
        estadoSecundario_2 = !estadoSecundario_2;

        if(spawn != 10)
          spawn++;
        else {
          if(campo)
            spawn = 1;
          else
            spawn = 2;
        }
      }
      break;

    case 3:
      if(digitalRead(secundario_b3) != estadoSecundario_3) {
        if(spawn < 9)
          spawn += 2;
        else {
          if(campo)
            spawn = 1;
          else
            spawn = 2;
        }
        estadoSecundario_3 = !estadoSecundario_3;
      }
      break;

    case 4:
      if(digitalRead(secundario_b4) != estadoSecundario_4) {
        if(plan < numeroPlanes)
          plan++;
        else
          plan = 1;
        estadoSecundario_4 = !estadoSecundario_4;
      }
      break;
  }

  if(devolver == 2)
    display::pintarCampo(mylcd, campo, spawn);
  else if(devolver == 3)
    display::pintarSpawn(mylcd, campo, spawn);
  else if(devolver == 4)
    display::pintarPlan(mylcd, plan);
}

int ejecutarMenuBicho() {
  if(nuevaLinea) {
    nuevaLinea = false;
    return display::escribirErrores(mylcd, WHITE, 2, errores, 10, 10, menuEstadistica,
    menuCaballo, menuBicho, menuLidar, menuApagar, menuActual, secundario_b1);
  }
}

int ejecutarMenuLidar() {
  String apagar[6] = {"Seleccione apagar", "para detener el", "lidar", "Estado del lidar:", "Encendido", "Apagar"};
  String encender[6] = {"Seleccione", "encender para", "arrancar el lidar", "Estado del lidar:", "Apagado", "Encen."};
  int coordX = 8;
  int coordY[6] = {8, 45, 82, 200, 237, 290};
  int tamanno = 4;
  uint16_t color = WHITE;
  int devolver;

  for(int i = 0; i < 6; i++) {
    //Penultima y ultima vuelta
    if(i > 3 && i < 5) {
      if(lidar)
        color = GREEN;
      else
        color = RED;
    } else if(i > 4) {
      coordX = 4;
      tamanno--;
      color = WHITE;
    }

    if(lidar)
      display::escribirTexto(mylcd, color, tamanno, apagar[i], coordX, coordY[i]);
    else
      display::escribirTexto(mylcd, color, tamanno, encender[i], coordX, coordY[i]);

    if((devolver = mirarLidar()) != 0)
      return devolver;
  }
  return mirarLidar();
}

void ejecutarMenuApagar() {
  switch(checkButtons::mirarBotonesSecundario(secundario_b1, secundario_b2, secundario_b3, secundario_b4)) {
    case 1:
      Serial.write("Apagar robot");
      delay(1000);

      break;

    case 2:
      Serial.write("Reiniciar robot");
      delay(1000);

      break;
  }
}

int seleccionarMenu(int eleccion) {
  codigoInterrupcion = 0;

  if(menuDevuelto != 0)
    menuActual = menuDevuelto;

  if(digitalRead(secundario_b1))
    estadoSecundario_1 = true;
  if(digitalRead(secundario_b2))
    estadoSecundario_2 = true;
  if(digitalRead(secundario_b3))
    estadoSecundario_3 = true;
  if(digitalRead(secundario_b4))
    estadoSecundario_4 = true;

  switch(eleccion) {
    case 1:
      display::pintarIconos(mylcd, WHITE, GREEN, BLACK, BLUE, BLACK, YELLOW, BLACK, RED, BLACK, WHITE);
      display::escribirTexto(mylcd, WHITE, 3, "Reini.", 5, 290);

      if((codigoInterrupcion = ejecutarMenuEstadistica()) != 0) {
        menuDevuelto = codigoInterrupcion;
        return menuDevuelto;
      }

      break;

    case 2:
      display::pintarIconos(mylcd, BLACK, GREEN, WHITE, BLUE, BLACK, YELLOW, BLACK, RED, BLACK, WHITE);
      mylcd.Set_Draw_color(WHITE);

      display::pintarCampo(mylcd, campo, spawn);
      display::pintarPlan(mylcd, plan);

      display::escribirTexto(mylcd, WHITE, 3, "Listo", 12, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Equipo", 126, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Spawn", 256, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Plan", 388, 290);
      ejecutarMenuCaballo();

      break;

    case 3:
      nuevaLinea = true;
      display::pintarIconos(mylcd, BLACK, GREEN, BLACK, BLUE, WHITE, YELLOW, BLACK, RED, BLACK, WHITE);
      display::escribirTexto(mylcd, WHITE, 3, "Pausa", 13, 290);

      if((codigoInterrupcion = ejecutarMenuBicho()) != 0 && codigoInterrupcion != -1) {
        menuDevuelto = codigoInterrupcion;
        return menuDevuelto;
      }

      return display::escribirErrores(mylcd, WHITE, 2, errores, 10, 10, menuEstadistica,
      menuCaballo, menuBicho, menuLidar, menuApagar, menuActual, secundario_b1);

      break;

    case 4:
      display::pintarIconos(mylcd, BLACK, GREEN, BLACK, BLUE, BLACK, YELLOW, WHITE, RED, BLACK, WHITE);

      if((codigoInterrupcion = ejecutarMenuLidar()) != 0) {
        menuDevuelto = codigoInterrupcion;
        return menuDevuelto;
      }

      break;

    case 5:
      display::pintarIconos(mylcd, BLACK, GREEN, BLACK, BLUE, BLACK, YELLOW, BLACK, RED, WHITE, BLACK);
      display::escribirTexto(mylcd, WHITE, 4, "Seleccione para", 8, 8);
      display::escribirTexto(mylcd, WHITE, 4, "apagar o", 8, 45);
      display::escribirTexto(mylcd, WHITE, 4, "reiniciar el", 8, 82);
      display::escribirTexto(mylcd, WHITE, 4, "robot", 8, 119);
      display::escribirTexto(mylcd, WHITE, 3, "Apagar", 4, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Reini.", 127, 290);
      ejecutarMenuApagar();

      break;

    default:
      switch(menuActual) {
        case 1:
          if((codigoInterrupcion = ejecutarMenuEstadistica()) != 0) {
            menuDevuelto = codigoInterrupcion;
            return menuDevuelto;
          }

          break;

        case 2:
          ejecutarMenuCaballo();
          break;

        case 3:
          if((codigoInterrupcion = ejecutarMenuBicho()) != 0 && codigoInterrupcion != -1) {
            menuDevuelto = codigoInterrupcion;
            return menuDevuelto;
          }

          break;

        case 4:
          if((codigoInterrupcion = ejecutarMenuLidar()) != 0) {
            menuDevuelto = codigoInterrupcion;
            return menuDevuelto;
          }

          break;

        case 5:
          ejecutarMenuApagar();
          break;
      }
      break;
  }
  return 0;
}

void setup() {
  //Configuracion botones menu principal
  pinMode(menuEstadistica, INPUT_PULLUP);
  pinMode(menuCaballo, INPUT_PULLUP);
  pinMode(menuBicho, INPUT_PULLUP);
  pinMode(menuLidar, INPUT_PULLUP);
  pinMode(menuApagar, INPUT_PULLUP);

  //Configuracion botones menu secundario
  pinMode(secundario_b1, INPUT_PULLUP);
  pinMode(secundario_b2, INPUT_PULLUP);
  pinMode(secundario_b3, INPUT_PULLUP);
  pinMode(secundario_b4, INPUT_PULLUP);

  //Configuracion pantalla
  Serial.begin(9600);
  mylcd.Init_LCD();
  mylcd.Set_Rotation(-1);
  Serial.println(mylcd.Read_ID(), HEX);
  mylcd.Fill_Screen(BLACK);

  display::marcoMenuPrincipal(mylcd);
  mylcd.Set_Text_Back_colour(BLACK);
  seleccionarMenu(1);

}

void loop() {
  if(Serial.available())
    insertarError(Serial.readString());

  seleccionarMenu(menuDevuelto = checkButtons::mirarBotonesPrincipal(menuEstadistica,
  menuCaballo, menuBicho, menuLidar, menuApagar, menuActual));
  if(codigoInterrupcion != 0)
    seleccionarMenu(codigoInterrupcion);
}
