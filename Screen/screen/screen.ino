#include <LCDWIKI_GUI.h> //Core graphics library
#include <LCDWIKI_KBV.h> //Hardware-specific library

//if the IC model is known or the modules is unreadable,you can use this constructed function
LCDWIKI_KBV mylcd(ILI9488, A3, A2, A1, A0, A4); //model,cs,cd,wr,rd,reset
//if the IC model is not known and the modules is readable,you can use this constructed function
//LCDWIKI_KBV mylcd(320,480,A3,A2,A1,A0,A4);//width,height,cs,cd,wr,rd,reset

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

//Colores
#define BLACK   0x0000
#define BLUE    0x001F
#define GREY    0x000F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

byte buffer[13];

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

  marcoMenuPrincipal();
  seleccionarMenu(1);
}

//Devolvemos el boton pulsado con prioridad de menor a mayor
int mirarBotonesPrincipal() {
  if(!digitalRead(menuEstadistica) && menuActual != 1)
    return menuActual = 1;
  else if(!digitalRead(menuCaballo) && menuActual != 2)
    return menuActual = 2;
  else if(!digitalRead(menuBicho) && menuActual != 3)
    return menuActual = 3;
  else if(!digitalRead(menuLidar) && menuActual != 4)
    return menuActual = 4;
  else if(!digitalRead(menuApagar) && menuActual != 5)
    return menuActual = 5;
  else
    return 0;
}

//Hacemos lo mismo que en el metodo mirarBotonesPrincipal() pero con
//los botones del menu secundario
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

//Segun los atributos un fondo de un icono en blanco (seleccionado) y resto en negro
void pintarIconos(uint16_t estadistica_c1, uint16_t estadistica_c2,
uint16_t caballo_c1, uint16_t caballo_c2, uint16_t bicho_c1, uint16_t bicho_c2,
uint16_t lidar_c1, uint16_t lidar_c2, uint16_t apagar_c1, uint16_t apagar_c2) {
  //Fondo estadistica
  mylcd.Fill_Rect(432, 0, 48, 49, estadistica_c1);
  //Icono estadistica
  mylcd.Fill_Rect(436, 33, 8, 8, estadistica_c2);
  mylcd.Fill_Rect(440, 25, 8, 8, estadistica_c2);
  mylcd.Fill_Rect(444, 17, 8, 8, estadistica_c2);
  mylcd.Fill_Rect(448,  9, 8, 8, estadistica_c2);
  mylcd.Fill_Rect(456, 17, 8, 8, estadistica_c2);
  mylcd.Fill_Rect(460, 25, 8, 8, estadistica_c2);
  mylcd.Fill_Rect(468, 21, 8, 8, estadistica_c2);

  //Fondo caballo
  mylcd.Fill_Rect(432, 57, 48, 48, caballo_c1);
  //Icono caballo
  mylcd.Fill_Rect(440, 93, 32, 8, caballo_c2);
  mylcd.Fill_Rect(444, 89, 24, 4, caballo_c2);
  mylcd.Fill_Rect(448, 85, 24, 4, caballo_c2);
  mylcd.Fill_Rect(452, 81, 24, 4, caballo_c2);
  mylcd.Fill_Rect(436, 73, 40, 8, caballo_c2);
  mylcd.Fill_Rect(440, 69, 12, 4, caballo_c2);
  mylcd.Fill_Rect(456, 69, 16, 4, caballo_c2);
  mylcd.Fill_Rect(448, 65, 20, 4, caballo_c2);
  mylcd.Fill_Rect(452, 61,  8, 4, caballo_c2);

  //Fondo bicho
  mylcd.Fill_Rect(432, 113, 48, 48, bicho_c1);
  //Icono bicho
  mylcd.Fill_Rect(444, 153,  4, 4, bicho_c2);
  mylcd.Fill_Rect(452, 153, 12, 4, bicho_c2);
  mylcd.Fill_Rect(444, 149, 28, 4, bicho_c2);
  mylcd.Fill_Rect(436, 145,  4, 4, bicho_c2);
  mylcd.Fill_Rect(444, 145,  8, 4, bicho_c2);
  mylcd.Fill_Rect(464, 145,  8, 4, bicho_c2);
  mylcd.Fill_Rect(436, 141, 12, 4, bicho_c2);
  mylcd.Fill_Rect(460, 141,  4, 4, bicho_c2);
  mylcd.Fill_Rect(468, 141,  8, 4, bicho_c2);
  mylcd.Fill_Rect(440, 137,  8, 4, bicho_c2);
  mylcd.Fill_Rect(456, 137,  4, 4, bicho_c2);
  mylcd.Fill_Rect(468, 137,  8, 4, bicho_c2);
  mylcd.Fill_Rect(440, 133,  8, 4, bicho_c2);
  mylcd.Fill_Rect(452, 133,  4, 4, bicho_c2);
  mylcd.Fill_Rect(468, 133,  8, 4, bicho_c2);
  mylcd.Fill_Rect(436, 129, 16, 4, bicho_c2);
  mylcd.Fill_Rect(464, 129,  8, 4, bicho_c2);
  mylcd.Fill_Rect(436, 125, 40, 4, bicho_c2);
  mylcd.Fill_Rect(440, 121, 24, 4, bicho_c2);
  mylcd.Fill_Rect(444, 117,  8, 4, bicho_c2);
  mylcd.Fill_Rect(460, 117,  8, 4, bicho_c2);

  //Fondo lidar
  mylcd.Fill_Rect(432, 169, 48, 48, lidar_c1);
  //Icono lidar
  mylcd.Fill_Rect(436, 205, 40, 8, lidar_c2);
  mylcd.Fill_Rect(436, 201,  4, 4, lidar_c2);
  mylcd.Fill_Rect(472, 201,  4, 4, lidar_c2);
  mylcd.Fill_Rect(436, 197,  4, 4, lidar_c2);
  mylcd.Fill_Rect(444, 197, 24, 4, lidar_c2);
  mylcd.Fill_Rect(472, 197,  4, 4, lidar_c2);
  mylcd.Fill_Rect(436, 189,  8, 8, lidar_c2);
  mylcd.Fill_Rect(452, 189,  8, 8, lidar_c2);
  mylcd.Fill_Rect(468, 189,  8, 8, lidar_c2);
  mylcd.Fill_Rect(436, 185,  4, 4, lidar_c2);
  mylcd.Fill_Rect(444, 185, 24, 4, lidar_c2);
  mylcd.Fill_Rect(472, 185,  4, 4, lidar_c2);
  mylcd.Fill_Rect(440, 181,  4, 4, lidar_c2);
  mylcd.Fill_Rect(468, 181,  4, 4, lidar_c2);
  mylcd.Fill_Rect(444, 177,  4, 4, lidar_c2);
  mylcd.Fill_Rect(464, 177,  4, 4, lidar_c2);
  mylcd.Fill_Rect(448, 173, 16, 4, lidar_c2);

  //Fondo apagar
  mylcd.Fill_Rect(432, 225, 48, 48, apagar_c1);
  //Icono apagar
  mylcd.Fill_Rect(444, 265, 24, 4, apagar_c2);
  mylcd.Fill_Rect(440, 261,  8, 4, apagar_c2);
  mylcd.Fill_Rect(464, 261,  8, 4, apagar_c2);
  mylcd.Fill_Rect(436, 257,  8, 4, apagar_c2);
  mylcd.Fill_Rect(468, 257,  8, 4, apagar_c2);
  mylcd.Fill_Rect(436, 253,  4, 4, apagar_c2);
  mylcd.Fill_Rect(472, 253,  4, 4, apagar_c2);
  mylcd.Fill_Rect(436, 249,  4, 4, apagar_c2);
  mylcd.Fill_Rect(472, 249,  4, 4, apagar_c2);
  mylcd.Fill_Rect(436, 245,  4, 4, apagar_c2);
  mylcd.Fill_Rect(452, 245,  8, 4, apagar_c2);
  mylcd.Fill_Rect(472, 245,  4, 4, apagar_c2);
  mylcd.Fill_Rect(436, 241,  8, 4, apagar_c2);
  mylcd.Fill_Rect(452, 241,  8, 4, apagar_c2);
  mylcd.Fill_Rect(468, 241,  8, 4, apagar_c2);
  mylcd.Fill_Rect(440, 237,  8, 4, apagar_c2);
  mylcd.Fill_Rect(452, 237,  8, 4, apagar_c2);
  mylcd.Fill_Rect(464, 237,  8, 4, apagar_c2);
  mylcd.Fill_Rect(444, 233, 24, 4, apagar_c2);
  mylcd.Fill_Rect(452, 229,  8, 4, apagar_c2);
}

void marcoMenuPrincipal() {
  //Verticales menu principal
  mylcd.Fill_Rect(424, 0, 8, 280, WHITE);
  mylcd.Fill_Rect(49, 273, 8, 56, WHITE);

  //Horizontales menu principal
  mylcd.Fill_Rect(424,  49,  56, 8, WHITE);
  mylcd.Fill_Rect(424, 105,  56, 8, WHITE);
  mylcd.Fill_Rect(424, 161,  56, 8, WHITE);
  mylcd.Fill_Rect(424, 217,  56, 8, WHITE);
  mylcd.Fill_Rect(  0, 273, 480, 8, WHITE);
}

void pintarIconosConexion() {
  //Marco conexion
  mylcd.Fill_Rect(47, 0, 8, 55, WHITE);
  mylcd.Fill_Rect(0, 47, 47, 8, WHITE);

  //Conexion router
  mylcd.Fill_Rect(3, 30, 8, 13, GREY);
  mylcd.Fill_Rect(19, 17, 8, 26, GREY);
  mylcd.Fill_Rect(35, 4, 8, 39, GREY);
}

void pintarCampo() {
  //Verticales campo
  mylcd.Fill_Rect(416, 8, 8, 272, WHITE);
  mylcd.Fill_Rect(  8, 8, 8, 272, WHITE);

  //Horizontales campo
  mylcd.Fill_Rect(  8,   8,  400, 8, WHITE);
  mylcd.Fill_Rect(416, 416,  56, 8, WHITE);
}

void ejecutarMenuEstadistica() {
  //Pendiente
}

void ejecutarMenuCaballo() {
  switch(mirarBotonesSecundario()) {
    case 1:
      Serial.write("Robot listo para jugar");
      delay(1000);
      break;

    case 2:
      Serial.write("Elección de equipo");
      delay(1000);
      break;

    case 3:
      Serial.write("Elección de spawn");
      delay(1000);
      break;

    case 4:
      Serial.write("Elección de plan");
      delay(1000);
      break;
  }
}

void ejecutarMenuBicho() {
  //Pendiente
}

void ejecutarMenuLidar() {
  //Pendiente
}

void ejecutarMenuApagar() {
  switch(mirarBotonesSecundario()) {
    case 1:
      //Pendiente
      break;

    case 2:
      //Pendiente
      break;
  }
}

void seleccionarMenu(int eleccion) {
  switch(eleccion) {
    case 1:
      mylcd.Set_Text_Mode(0);      
      pintarIconos(WHITE, BLACK, BLACK, WHITE, BLACK, WHITE, BLACK, WHITE, BLACK, WHITE);
      ejecutarMenuEstadistica();

      mylcd.Set_Text_colour(WHITE);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(1);
      mylcd.Print_String("Hello World!", 0, 0);
      escribirTexto(WHITE, 2, "Hello World!", 0, 40);
      escribirTexto(WHITE, 3, "Hello World!", 0, 104);
      escribirTexto(WHITE, 4, "Hello!", 0, 192);
      escribirTexto(WHITE, 5, "Hello!", 0, 224);

      break;

    case 2:
      mylcd.Set_Text_Mode(0);
      pintarIconos(BLACK, WHITE, WHITE, BLACK, BLACK, WHITE, BLACK, WHITE, BLACK, WHITE);
      ejecutarMenuCaballo();

      mylcd.Set_Text_colour(WHITE);
      mylcd.Set_Text_Back_colour(BLACK);
      mylcd.Set_Text_Size(1);
      mylcd.Print_String("Hello World!", 0, 0);
      escribirTexto(WHITE, 2, "Hello World!", 0, 40);
      escribirTexto(WHITE, 3, "Hello World!", 0, 104);
      escribirTexto(WHITE, 4, "Hello!", 0, 192);
      escribirTexto(WHITE, 5, "Hello!", 0, 224);

      break;

    case 3:
      pintarIconos(BLACK, WHITE, BLACK, WHITE, WHITE, BLACK, BLACK, WHITE, BLACK, WHITE);
      ejecutarMenuBicho();

      break;

    case 4:
      pintarIconos(BLACK, WHITE, BLACK, WHITE, BLACK, WHITE, WHITE, BLACK, BLACK, WHITE);
      ejecutarMenuLidar();

      break;

    case 5:
      pintarIconos(BLACK, WHITE, BLACK, WHITE, BLACK, WHITE, BLACK, WHITE, WHITE, BLACK);
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
          ejecutarMenuBicho();
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
}

void escribirTexto(uint16_t color, uint8_t tamanno, String texto, uint8_t coordenada_X, uint8_t coordenada_Y) {
  mylcd.Set_Text_colour(color);
  mylcd.Set_Text_Size(tamanno);
  mylcd.Print_String(texto, coordenada_X, coordenada_Y);
}

void pintarMatriz(int matriz[][2], uint16_t color) {
  for(int i = 0; i < (sizeof(matriz) / sizeof(matriz[0])); i++) {
    mylcd.Draw_Pixe(matriz[i][0], matriz[i][1], color);
  }
}

void loop() {
  seleccionarMenu(mirarBotonesPrincipal());
}