#pragma once

#include "display.hpp"
#include "io.hpp"
#include "reactive.hpp"
#include "utils.hpp"

#include <Vector.h>

namespace menus {
  struct ApplicationContext;

  void stats_menu_start(ApplicationContext&);
  void strat_menu_start(ApplicationContext&);
  void debug_menu_start(ApplicationContext&);
  void lidar_menu_start(ApplicationContext&);
  void power_menu_start(ApplicationContext&);

  void stats_menu_update(ApplicationContext&);
  void strat_menu_update(ApplicationContext&);
  void debug_menu_update(ApplicationContext&);
  void lidar_menu_update(ApplicationContext&);
  void power_menu_update(ApplicationContext&);

  struct ContextMenuEntry {
    void(*callback)(ApplicationContext&);
    const char* display_name;
    bool present = false;
    ContextMenuEntry() = default;
    ContextMenuEntry(const char* display_name)
      : callback(nullptr), display_name(display_name), present(true) {}
    ContextMenuEntry(const char* display_name, void(*callback)(ApplicationContext&))
      : callback(callback), display_name(display_name), present(true) {}
  };

  struct PrimaryMenuEntry {
    void(*on_enter)(ApplicationContext&);
    void(*update)(ApplicationContext&);
    ContextMenuEntry* context_menus;
  };

  struct ApplicationContext {
    LCDWIKI_KBV lcd;
    PrimaryMenuEntry primary_menus[5];
    PrimaryMenuEntry* current_menu;

    // Reactive variables
    reactive::Int score;
    reactive::Bool playzone = true;
    reactive::Int starting_position = 1;
    reactive::Int plan = 1;
    reactive::Bool lidar = true;
    reactive::Bool pendrive_plugged = false;
    reactive::Bool config_confirmed = false;

    // BT list
    String ___bt_list_arr[10];
    Vector<String> bt_list = Vector<String>(___bt_list_arr);
    reactive::Int selected_bt = 0;
  };

  ContextMenuEntry* stats_ctx_menus();
  ContextMenuEntry* strat_ctx_menus();
  ContextMenuEntry* debug_ctx_menus();
  ContextMenuEntry* lidar_ctx_menus();
  ContextMenuEntry* power_ctx_menus();
}
