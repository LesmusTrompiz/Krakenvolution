#include "display.hpp"
#include "menus.hpp"

void menus::stats_menu_start(menus::ApplicationContext& ctx) {
  ctx.score.trigger();
}

void menus::stats_menu_update(ApplicationContext & ctx) {
  if (reactive::CHECK || ctx.score) {
    String formatted = "Score: ";
    formatted += ctx.score;
    ctx.lcd.Fill_Rect(0, 0, 100, 30, display::BLACK);
    display::escribirTexto(ctx.lcd, display::WHITE, 2, formatted, 0, 0);
  }
}


void test_callback();
menus::ContextMenuEntry stats_menus[4] {
  menus::ContextMenuEntry("UwU"),
  menus::ContextMenuEntry("Nya"),
  menus::ContextMenuEntry("Chetos"),
  menus::ContextMenuEntry(":)"),
};

menus::ContextMenuEntry* menus::stats_ctx_menus() {
  stats_menus[0].callback = [](ApplicationContext& ctx) {
    ctx.score = ctx.score + 1;
  };

  return stats_menus;
}
