#pragma once

#include "display.hpp"
#include "io.hpp"
#include "reactive.hpp"
#include "utils.hpp"

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
    reactive::Bool playzone;
    reactive::Int starting_position = 1;
  };

  ContextMenuEntry* stats_ctx_menus();
  ContextMenuEntry* strat_ctx_menus();
  ContextMenuEntry* debug_ctx_menus();
  ContextMenuEntry* lidar_ctx_menus();
  ContextMenuEntry* power_ctx_menus();
}
