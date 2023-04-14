#include "menus.hpp"

void menus::debug_menu_start(menus::ApplicationContext& ctx) {

}

void menus::debug_menu_update(ApplicationContext &) {

}

menus::ContextMenuEntry debug_menus[4] = {
  menus::ContextMenuEntry("")
};

menus::ContextMenuEntry* menus::debug_ctx_menus() {
  return debug_menus;
}
