#include "LCDWIKI_KBV.h"
#include "LCDWIKI_GUI.h"
#include "display.hpp"
#include "checkButtons.hpp"
#include "logicalStates.hpp"

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV mylcd(ILI9488, A3, A2, A1, A0, A4); //model,cs,cd,wr,rd,reset
//if the IC model is not known and the modules is readable,you can use this constructed function
//LCDWIKI_KBV mylcd(320,480,A3,A2,A1,A0,A4);//width,height,cs,cd,wr,rd,reset

//Colores
#define BLACK       0x0000
#define BLUE        0x007E
#define RED         0xF800
#define GREEN       0x07E0
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF

String errores[13];

int recibirMensajes() {
  String mensaje;

  if(Serial.available()) {
    mensaje = Serial.readStringUntil(':');

    if(mensaje.equals("error")) {
      display::ordenarErrores(errores, Serial.readStringUntil('\n'));
      return 1;
    }
  }
  return 0;
}

int ejecutarMenuEstadistica() {
  int devolver;
  display::escribirTexto(mylcd, WHITE, 5, "Tiempo: 62s", 8, 8);
  display::escribirTexto(mylcd, WHITE, 5, "Tartas: 4", 8, 55);

  if((devolver = checkButtons::mirarBotonesPrincipal(logicalStates::getMenuActual())) != 0)
    return devolver;

  display::escribirTexto(mylcd, WHITE, 5, "Puntos: 32", 8, 102);
  display::escribirTexto(mylcd, MAGENTA, 6, "UAHRKrakens", 8, 223);

  return 0;
}

void ejecutarMenuCaballo() {
  int devolver;
  String colorCampo = "green";

  switch(devolver = checkButtons::mirarBotonesSecundario()) {
    case 1:
      if(!logicalStates::getCampo())
        colorCampo = "blue";

      if(digitalRead(checkButtons::getSecundario_b1()) != checkButtons::getEstadoSecundario_1()) {
        Serial.print("{\"info\":{\"field\":" + colorCampo + String(",\"spawn\":") + logicalStates::getSpawn() +
        String(",\"plan\":") + logicalStates::getPlan() + String("}}"));
        checkButtons::setEstadoSecundario_1(!checkButtons::getEstadoSecundario_1());
      }
      break;

    case 2:
      if(digitalRead(checkButtons::getSecundario_b2()) != checkButtons::getEstadoSecundario_2()) {
        logicalStates::setCampo(!logicalStates::getCampo());
        checkButtons::setEstadoSecundario_2(!checkButtons::getEstadoSecundario_2());
        logicalStates::setSpawnB2();
      }
      break;

    case 3:
      if(digitalRead(checkButtons::getSecundario_b3()) != checkButtons::getEstadoSecundario_3()) {
        logicalStates::setSpawnB3();
        checkButtons::setEstadoSecundario_3(!checkButtons::getEstadoSecundario_3());
      }
      break;

    case 4:
      if(digitalRead(checkButtons::getSecundario_b4()) != checkButtons::getEstadoSecundario_4()) {
        logicalStates::setPlan();
        checkButtons::setEstadoSecundario_4(!checkButtons::getEstadoSecundario_4());
      }
      break;
  }

  if(devolver == 2)
    display::pintarCampo(mylcd, logicalStates::getCampo(), logicalStates::getSpawn());
  else if(devolver == 3)
    display::pintarSpawn(mylcd, logicalStates::getCampo(), logicalStates::getSpawn());
  else if(devolver == 4)
    display::pintarPlan(mylcd, logicalStates::getPlan());
}

int ejecutarMenuBicho(boolean primeraVuelta) {
  if(recibirMensajes() == 1 || primeraVuelta)
    return display::escribirErrores(mylcd, WHITE, 2, errores, 10, 10, logicalStates::getMenuActual(), checkButtons::getSecundario_b1());
  return 0;
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
      if(logicalStates::getLidar())
        color = GREEN;
      else
        color = RED;
    } else if(i > 4) {
      coordX = 4;
      tamanno--;
      color = WHITE;
    }

    if(logicalStates::getLidar()) {
      Serial.write("{\"command\":{\"name\":turn_lidar,\"arg\":false}}");
      display::escribirTexto(mylcd, color, tamanno, apagar[i], coordX, coordY[i]);
    } else  {
      Serial.write("{\"command\":{\"name\":turn_lidar,\"arg\":true}}");
      display::escribirTexto(mylcd, color, tamanno, encender[i], coordX, coordY[i]);
    }

    if((devolver = checkButtons::mirarLidar(mylcd, logicalStates::getMenuActual(), BLACK)) != 0)
      return devolver;
  }
  return checkButtons::mirarLidar(mylcd, logicalStates::getMenuActual(), BLACK);
}

void ejecutarMenuApagar() {
  switch(checkButtons::mirarBotonesSecundario()) {
    case 1:
      Serial.write("{\"command\":{\"name\":turn_robot,\"arg\":true}}");
      delay(1000);

      break;

    case 2:
      Serial.write("{\"command\":{\"name\":reboot_robot,\"arg\":true}}");
      delay(1000);

      break;
  }
}

int seleccionarMenu(int eleccion) {
  if(logicalStates::getMenuDevuelto() != 0)
    logicalStates::setMenuActual(logicalStates::getMenuDevuelto());
  logicalStates::setMenuDevuelto(0);

  if(digitalRead(checkButtons::getSecundario_b1()))
    checkButtons::setEstadoSecundario_1(true);
  if(digitalRead(checkButtons::getSecundario_b2()))
    checkButtons::setEstadoSecundario_2(true);
  if(digitalRead(checkButtons::getSecundario_b3()))
    checkButtons::setEstadoSecundario_3(true);
  if(digitalRead(checkButtons::getSecundario_b4()))
    checkButtons::setEstadoSecundario_4(true);

  switch(eleccion) {
    case 1:
      display::pintarIconos(mylcd, WHITE, GREEN, BLACK, BLUE, BLACK, YELLOW, BLACK, RED, BLACK, WHITE);

      if(logicalStates::setMenuDevuelto(ejecutarMenuEstadistica()) != 0)
        return logicalStates::getMenuDevuelto();

      break;

    case 2:
      display::pintarIconos(mylcd, BLACK, GREEN, WHITE, BLUE, BLACK, YELLOW, BLACK, RED, BLACK, WHITE);
      mylcd.Set_Draw_color(WHITE);

      display::pintarCampo(mylcd, logicalStates::getCampo(), logicalStates::getSpawn());
      display::pintarPlan(mylcd, logicalStates::getPlan());

      display::escribirTexto(mylcd, WHITE, 3, "Listo", 12, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Equipo", 126, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Spawn", 256, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Plan", 388, 290);
      ejecutarMenuCaballo();

      break;

    case 3:
      display::pintarIconos(mylcd, BLACK, GREEN, BLACK, BLUE, WHITE, YELLOW, BLACK, RED, BLACK, WHITE);
      display::escribirTexto(mylcd, WHITE, 3, "Pausa", 13, 290);

      if(logicalStates::setMenuDevuelto(ejecutarMenuBicho(true)) != 0)
        return logicalStates::getMenuDevuelto();

      break;

    case 4:
      display::pintarIconos(mylcd, BLACK, GREEN, BLACK, BLUE, BLACK, YELLOW, WHITE, RED, BLACK, WHITE);

      if(logicalStates::setMenuDevuelto(ejecutarMenuLidar()) != 0)
        return logicalStates::getMenuDevuelto();

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
      switch(logicalStates::getMenuActual()) {
        case 1:
          if(logicalStates::setMenuDevuelto(ejecutarMenuEstadistica()) != 0)
            return logicalStates::getMenuDevuelto();

          break;

        case 2:
          ejecutarMenuCaballo();
          break;

        case 3:
          if(logicalStates::setMenuDevuelto(ejecutarMenuBicho(false)) != 0)
            return logicalStates::getMenuDevuelto();

          break;

        case 4:
          if(logicalStates::setMenuDevuelto(ejecutarMenuLidar()) != 0)
            return logicalStates::getMenuDevuelto();

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
  pinMode(checkButtons::getMenuEstadistica(), INPUT_PULLUP);
  pinMode(checkButtons::getMenuCaballo(), INPUT_PULLUP);
  pinMode(checkButtons::getMenuBicho(), INPUT_PULLUP);
  pinMode(checkButtons::getMenuLidar(), INPUT_PULLUP);
  pinMode(checkButtons::getMenuApagar(), INPUT_PULLUP);

  //Configuracion botones menu secundario
  pinMode(checkButtons::getSecundario_b1(), INPUT_PULLUP);
  pinMode(checkButtons::getSecundario_b2(), INPUT_PULLUP);
  pinMode(checkButtons::getSecundario_b3(), INPUT_PULLUP);
  pinMode(checkButtons::getSecundario_b4(), INPUT_PULLUP);

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
  seleccionarMenu(logicalStates::setMenuDevuelto(checkButtons::mirarBotonesPrincipal(logicalStates::getMenuActual())));
  if(logicalStates::getMenuDevuelto() != 0)
    seleccionarMenu(logicalStates::getMenuDevuelto());
}
