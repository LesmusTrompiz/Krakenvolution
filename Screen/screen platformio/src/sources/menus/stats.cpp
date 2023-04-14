#include "menus.hpp"

void menus::stats_menu_start(menus::ApplicationContext& ctx) {

}

void menus::stats_menu_update(ApplicationContext &) {
}

void test_callback();
menus::ContextMenuEntry stats_menus[4] {
  menus::ContextMenuEntry("UwU"),
  menus::ContextMenuEntry("Nya", test_callback),
  menus::ContextMenuEntry("Chetos"),
  menus::ContextMenuEntry(":)"),
};

void test_callback() {

}

menus::ContextMenuEntry* menus::stats_ctx_menus() {
  stats_menus[0].callback = []() {

  };

  return stats_menus;
}
