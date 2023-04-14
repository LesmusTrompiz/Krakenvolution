#include "menus.hpp"

void menus::strat_menu_start(menus::ApplicationContext& ctx) {

}

void menus::strat_menu_update(ApplicationContext& ctx) {
  static utils::Timer t(500);
  static bool colored = false;
  if (t.triggered()) {
    ctx.lcd.Fill_Rect(10, 10, 20, 20, colored ? 
        display::rgb_to_565(255, 0, 0) : display::rgb_to_565(0, 255, 0));
    colored = !colored;
  }
}

menus::ContextMenuEntry strat_menus[4] = {
  menus::ContextMenuEntry("")
};

menus::ContextMenuEntry* menus::strat_ctx_menus() {
  return strat_menus;
}
