#include "menus.hpp"

void menus::lidar_menu_start(menus::ApplicationContext& ctx) {

}

void menus::lidar_menu_update(ApplicationContext &) {

}


menus::ContextMenuEntry lidar_menus[4] = {
  menus::ContextMenuEntry("")
};

menus::ContextMenuEntry* menus::lidar_ctx_menus() {
  return lidar_menus;
}
