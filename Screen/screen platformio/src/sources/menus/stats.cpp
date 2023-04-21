#include "display.hpp"
#include "menus.hpp"

void menus::stats_menu_start(menus::ApplicationContext& ctx) {
  ctx.score.trigger();
  ctx.config_confirmed.trigger();
}

void menus::stats_menu_update(ApplicationContext & ctx) {
  if (reactive::CHECK || ctx.score) {
    constexpr size_t SCORE_SIZE = 15;
    String formatted = "";
    formatted += ctx.score;
    size_t score_x_pos = (415 / 2) - ((6 * SCORE_SIZE * formatted.length() - 1) / 2);
    size_t score_y_pos = 268 / 2 - (SCORE_SIZE * 8 - 1) / 2;
    ctx.lcd.Fill_Rect(0, score_y_pos, 415, SCORE_SIZE * 8 - 1, display::BLACK);
    display::escribirTexto(ctx.lcd, display::WHITE, SCORE_SIZE, formatted, score_x_pos, score_y_pos);
  }

  if (reactive::CHECK || ctx.config_confirmed) {
    ctx.lcd.Fill_Rect(0, 268 - 3 * 8 - 1, 415, 3 * 8 - 1, display::BLACK);
    ctx.lcd.Set_Text_colour(ctx.config_confirmed ? display::GREEN:display::RED);
    ctx.lcd.Set_Text_Size(3);
    String txt = ctx.config_confirmed ? "Valid":"Invalid";
    txt += " configuration";
    ctx.lcd.Print_String(txt, 0, 268 - 3 * 8 - 1 );
    ctx.lcd.Set_Text_Back_colour(display::BLACK);
    ctx.lcd.Set_Text_Mode(display::DRAW_BG); 
    display::escribirTexto(ctx.lcd, display::WHITE, 3, txt, 8, 8);
  }
}

void validate_config(menus::ApplicationContext& ctx) {
  if (ctx.pendrive_plugged && ctx.bt_list.size()) {
    ctx.config_confirmed = true;
  }
}

void test_callback();
menus::ContextMenuEntry stats_menus[4] {
  menus::ContextMenuEntry("Valid", &validate_config),
  menus::ContextMenuEntry(""),
  menus::ContextMenuEntry(""),
  menus::ContextMenuEntry(""),
};

menus::ContextMenuEntry* menus::stats_ctx_menus() {
  return stats_menus;
}
