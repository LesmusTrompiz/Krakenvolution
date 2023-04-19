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

  context.current_menu = &context.primary_menus[1];
  io::force_primary(1);

  size_t startup_font_size = 3;
  context.lcd.Set_Text_Mode(true);
  context.lcd.Set_Text_colour(display::WHITE);
  context.lcd.Set_Text_Size(startup_font_size);
  String startup_msg = "Waiting for node...";
  context.lcd.Print_String(startup_msg, 
                           context.lcd.Get_Width() / 2 - (((6 * startup_font_size - 1) * startup_msg.length()) / 2),
                           context.lcd.Get_Height() / 2 - ((8 * startup_font_size - 1) / 2)
                           );

  bool node_ready = false;
  bool color = true;
  int dots_counter = 0;
  while (!node_ready) {
    context.lcd.Set_Draw_color(color ? display::WHITE:display::BLACK);
    switch (dots_counter) {
      case 0:
        context.lcd.Fill_Circle(context.lcd.Get_Width() / 2 - 40, context.lcd.Get_Height() / 2 + 40, 6);
        break;
      case 1:
        context.lcd.Fill_Circle(context.lcd.Get_Width() / 2, context.lcd.Get_Height() / 2 + 40, 6);
        break;
      case 2:
        context.lcd.Fill_Circle(context.lcd.Get_Width() / 2 + 40, context.lcd.Get_Height() / 2 + 40, 6);
        break;
    }

    ++dots_counter %= 3;
    if (dots_counter == 0) {
      color = !color;
    }

    if (dots_counter == 0 && color)
      Serial.println("ready");

    if (Serial.available()) {
      String msg = Serial.readStringUntil('\n');
      if (msg.equals("ok"))
        node_ready = true;
    }
    delay(100);
  }
}

void loop() {
  static bool first_update = true;


  if (auto primary_buttons_event = io::pressed_primary_index()) {
    static auto last_pressed = -1;
    
    if (primary_buttons_event.unwrap() != last_pressed) {
      // Change menu
      context.current_menu = &context.primary_menus[primary_buttons_event.unwrap()];
      
      // context.lcd.Fill_Screen(display::BLACK);
      display::marcoMenuPrincipal(context.lcd);
      display::pintarIconos(context.lcd, primary_buttons_event.unwrap());

      context.lcd.Set_Text_Back_colour(display::WHITE);
      for (int i = 0; i < 4; i++) {
        auto& ctx_menu = context.current_menu->context_menus[i];
        if (ctx_menu.present) {
          display::escribirTexto(context.lcd, display::WHITE, 3, ctx_menu.display_name, 12 + 120 * i, 290);
        }
      }

      context.current_menu->on_enter(context);
    }

    last_pressed = primary_buttons_event.unwrap();
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
    bool valid = true;
    if (cmd.equals("score")) {
      context.score = arg.toInt();
    } else if (cmd.equals("pendrive")) {
      context.pendrive_plugged = arg.equals("true");
      if (!context.pendrive_plugged && context.config_confirmed) {
        // Send configuration to robot uwu
        String formatted = "{\"play_side\":\""; // tree play_site spawn
        formatted += context.playzone ? "green":"blue";
        formatted += "\",\"spawn\":";
        formatted += context.starting_position;
        formatted += ",\"tree\":\"";
        formatted += context.bt_list[context.selected_bt];
        formatted += "\"}";
        Serial.println(formatted);
        context.config_confirmed = false;
      }
    } else if (cmd.equals("plan")) {
      context.bt_list.push_back(arg);
      if (context.bt_list.size() == 1)
        context.selected_bt.trigger();
    } else if (cmd.equals("clear_plans")) {
      context.bt_list.clear();
      context.selected_bt.trigger();
      context.config_confirmed = false;
    } else {
      valid = false;
    }
    Serial.println(valid ? "ok":"nok");
  }

  context.current_menu->update(context);
}
