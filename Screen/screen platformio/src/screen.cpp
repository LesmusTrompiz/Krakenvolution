#include <LCDWIKI_GUI.h>
#include <LCDWIKI_KBV.h>
#include "../../include/display.hpp"
#include "../../include/checkButtons.hpp"
#include "headers/protocol.hpp"

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV mylcd(ILI9488, A3, A2, A1, A0, A4); //model,cs,cd,wr,rd,reset
//if the IC model is not known and the modules is readable,you can use this constructed function
//LCDWIKI_KBV mylcd(320,480,A3,A2,A1,A0,A4);//width,height,cs,cd,wr,rd,reset

//Colores
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

//Pines menu principal
int menuEstadistica = 25;
int menuCaballo = 24;
int menuBicho = 33;
int menuLidar = 39;
int menuApagar = 47;
int menuActual = 1;

//Pines menu secundario
int secundario_b1 = 41;
int secundario_b2 = 53;
int secundario_b3 = 51;
int secundario_b4 = 52;

//Estado botones secundarios
boolean estadoSecundario_1 = true;
boolean estadoSecundario_2 = true;
boolean estadoSecundario_3 = true;
boolean estadoSecundario_4 = true;

//Variables para las interrupciones
int menuDevuelto;
int codigoInterrupcion = 0;

boolean lidar = true;

/**ProtocolSM protocol_sm;
uahruart::serial::ClientProtocolBuffer buffer(&protocol_sm);

#if defined(USART_RX_vect)
  ISR(USART_RX_vect)
#elif defined(USART0_RX_vect)
  ISR(USART0_RX_vect)
#elif defined(USART_RXC_vect)
  ISR(USART_RXC_vect) // ATmega8
#else
  #error "Don't know what the Data Received vector is called for Serial"
#endif
  {
    buffer._rx_complete_irq();
  }

#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#else
  #error "Don't know what the Data Register Empty vector is called for Serial"
#endif
{
  buffer._tx_udr_empty_irq();
}*/

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

    return 3;
  }
  return 0;
}

void ejecutarMenuEstadistica() {
  //Pendiente
}

void ejecutarMenuCaballo() {
  switch(checkButtons::mirarBotonesSecundario(secundario_b1, secundario_b2, secundario_b3, secundario_b4)) {
    case 1:
      if(digitalRead(secundario_b1) != estadoSecundario_1) {
        Serial.write("Robot listo para jugar");
        estadoSecundario_1 = !estadoSecundario_1;
      }
      break;

    case 2:
      if(digitalRead(secundario_b2) != estadoSecundario_2) {
        Serial.write("Elección de equipo");
        estadoSecundario_2 = !estadoSecundario_2;
      }
      break;

    case 3:
      if(digitalRead(secundario_b3) != estadoSecundario_3) {
        Serial.write("Elección de spawn");
        estadoSecundario_3 = !estadoSecundario_3;
      }
      break;

    case 4:
      if(digitalRead(secundario_b4) != estadoSecundario_4) {
        Serial.write("Elección de plan");
        estadoSecundario_4 = !estadoSecundario_4;
      }
      break;
  }
}

int ejecutarMenuBicho() {
  return display::escribirErrores(mylcd, WHITE, 2, "Albion online es un mmorpg no lineal en el que escribes tu propia historia sin limitarte a seguir un camino prefijado, explora un amplio mundo abierto con cinco biomas unicos, todo cuanto hagas tendra su repercusion en el mundo, con su economia orientada al jugador de albion los jugadores crean practicamente todo el equipo a partir de los recursos que consiguen, el equipo que llevas define quien eres, cambia de arma y armadura para pasar de caballero a mago o juego como una mezcla de ambas clases, aventurate en el mundo abierto y haz frente a los habitantes y las criaturas de albion", 10, 10, menuEstadistica, menuCaballo, menuBicho, menuLidar, menuApagar, menuActual);
}

int ejecutarMenuLidar() {
  String apagar[6] = {"Seleccione apagar", "para detener el", "lidar", "Estado del lidar:", "Encendido", "Apagar"};
  String encender[6] = {"Seleccione", "encender para", "arrancar el lidar", "Estado del lidar:", "Apagado", "Encen."};
  int coordX = 8;
  int coordY[6] = {8, 45, 82, 200, 237, 290};
  int tamanno = 4;
  uint16_t color = WHITE;

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

    if(mirarLidar() == 3)
      return 3;
  }
  return mirarLidar();
}

void ejecutarMenuApagar() {
  switch(checkButtons::mirarBotonesSecundario(secundario_b1, secundario_b2, secundario_b3, secundario_b4)) {
    case 1:
      Serial.write("Reiniciar robot");
      delay(1000);

      break;

    case 2:
      Serial.write("Apagar robot");
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
      ejecutarMenuEstadistica();

      mylcd.Set_Text_colour(WHITE);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(1);
      mylcd.Print_String("Hello World!", 0, 0);
      display::escribirTexto(mylcd, WHITE, 2, "Hello World!", 0, 40);
      display::escribirTexto(mylcd, WHITE, 3, "Hello World!", 0, 104);
      display::escribirTexto(mylcd, WHITE, 4, "Hello!", 0, 192);
      display::escribirTexto(mylcd, WHITE, 5, "Hello!", 0, 224);

      break;

    case 2:
      display::pintarIconos(mylcd, BLACK, GREEN, WHITE, BLUE, BLACK, YELLOW, BLACK, RED, BLACK, WHITE);
      display::escribirTexto(mylcd, WHITE, 3, "Listo", 12, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Equipo", 126, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Spawn", 256, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Plan", 388, 290);
      ejecutarMenuCaballo();

      mylcd.Set_Text_colour(WHITE);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(1);
      mylcd.Print_String("Hola buenos días", 0, 0);
      display::escribirTexto(mylcd, WHITE, 2, "Hello World!", 0, 40);
      display::escribirTexto(mylcd, WHITE, 3, "Hello World!", 0, 104);
      display::escribirTexto(mylcd, WHITE, 4, "Hello!", 0, 192);
      display::escribirTexto(mylcd, WHITE, 5, "Hello!", 0, 220);

      break;

    case 3:
      display::pintarIconos(mylcd, BLACK, GREEN, BLACK, BLUE, WHITE, YELLOW, BLACK, RED, BLACK, WHITE);
      display::escribirTexto(mylcd, WHITE, 3, "Pausa", 13, 290);
      if((codigoInterrupcion = ejecutarMenuBicho()) != 0) {
        menuDevuelto = codigoInterrupcion;
        return menuDevuelto;
      }

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
      display::escribirTexto(mylcd, WHITE, 3, "Apagar", 4, 290);
      display::escribirTexto(mylcd, WHITE, 3, "Reini.", 127, 290);
      ejecutarMenuApagar();

      break;

    default:
      switch(menuActual) {
        case 1:
          ejecutarMenuEstadistica();
          break;

        case 2:
          ejecutarMenuCaballo();
          break;

        case 3:
          if((codigoInterrupcion = ejecutarMenuBicho()) != 0) {
            menuDevuelto = codigoInterrupcion;
            return menuDevuelto;
          }

          break;

        case 4:
          ejecutarMenuLidar();
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
  seleccionarMenu(menuDevuelto = checkButtons::mirarBotonesPrincipal(menuEstadistica,
  menuCaballo, menuBicho, menuLidar, menuApagar, menuActual));
  if(codigoInterrupcion != 0)
    seleccionarMenu(codigoInterrupcion);
}