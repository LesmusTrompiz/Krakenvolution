#include "menus.hpp"

void menus::power_menu_start(menus::ApplicationContext& ctx) {

}

void menus::power_menu_update(ApplicationContext &) {

}


menus::ContextMenuEntry power_menus[4] = {
  menus::ContextMenuEntry("")
};

menus::ContextMenuEntry* menus::power_ctx_menus() {
  return power_menus;
}
