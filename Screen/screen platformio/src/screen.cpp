#include <Arduino.h>
#include "io.hpp"
#include "menus.hpp"
#include "reactive.hpp"
#include "utils.hpp"
#include "display.hpp"

//if the IC model is known or the modules is unreadable,you can use this constructed function

menus::ApplicationContext context {
  LCDWIKI_KBV(ILI9488, A3, A2, A1, A0, A4),
  {
    menus::PrimaryMenuEntry {
      &menus::stats_menu_start,
      &menus::stats_menu_update,
      menus::stats_ctx_menus()
    },
    menus::PrimaryMenuEntry {
      &menus::strat_menu_start,
      &menus::strat_menu_update,
      menus::strat_ctx_menus()
    },
    menus::PrimaryMenuEntry {
      &menus::debug_menu_start,
      &menus::debug_menu_update,
      menus::debug_ctx_menus()
    },
    menus::PrimaryMenuEntry {
      &menus::lidar_menu_start,
      &menus::lidar_menu_update,
      menus::lidar_ctx_menus()
    },
    menus::PrimaryMenuEntry {
      &menus::power_menu_start,
      &menus::power_menu_update,
      menus::power_ctx_menus()
    }
  },
};

// Update with 50Hz timer
void tick() {
  io::tick();
}

void setup() {
  //Configuracion pantalla
  Serial.begin(115200);
  context.lcd.Init_LCD();
  context.lcd.Set_Rotation(-1);
  context.lcd.Fill_Screen(display::BLACK);

  // Register interrupts
  utils::setup(&tick);
  io::setup();

  context.current_menu = &context.primary_menus[0];

  context.current_menu->on_enter(context);
}

void loop() {
  if (auto e = io::pressed_primary_index()) {
    static auto last_pressed = -1;
    
    if (e.unwrap() != last_pressed) {
      // Change menu
      context.current_menu = &context.primary_menus[e.unwrap()];
      Serial.print("Se ha cambiado al menú ");
      Serial.println(e.unwrap());
      
      context.lcd.Fill_Screen(display::BLACK);
      display::marcoMenuPrincipal(context.lcd);
      display::pintarIconos(context.lcd, e.unwrap());

      context.lcd.Set_Text_Back_colour(display::WHITE);
      for (int i = 0; i < 4; i++) {
        auto& ctx_menu = context.current_menu->context_menus[i];
        if (ctx_menu.present) {
          display::escribirTexto(context.lcd, display::WHITE, 3, ctx_menu.display_name, 12 + 120 * i, 290);
        }
      }

      context.current_menu->on_enter(context);
    }

    last_pressed = e.unwrap();
  }

  if (auto e = io::pressed_context_index()) {
    // Run contextual command
    auto& ctx_menu = context.current_menu->context_menus[e.unwrap()];
    // Check if the callback and the contextual menu are present
    if (ctx_menu.present && (ctx_menu.callback != nullptr)) { 
      ctx_menu.callback(context);
    }
  }

  // Check if commands are available
  if (Serial.available()) {
    String unparsed = Serial.readStringUntil('\n');
    String cmd = "";
    String arg = "";
    bool reading_cmd = true;
    for (char c : unparsed) {
      if (c == ':')
        reading_cmd = false;
      else if (reading_cmd)
        cmd.concat(c);
      else
        arg.concat(c);
    }

    cmd.trim();
    arg.trim();

    // Interpret message
    if (cmd.equals("score")) {
      context.score = arg.toInt();
    }
  }

  context.current_menu->update(context);
}
