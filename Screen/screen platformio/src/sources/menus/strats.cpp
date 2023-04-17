#include "display.hpp"
#include "menus.hpp"

void menus::strat_menu_start(menus::ApplicationContext& ctx) {
  ctx.playzone.trigger();
  ctx.starting_position.trigger();
  ctx.plan.trigger();
  ctx.config_confirmed.trigger();
}

void menus::strat_menu_update(ApplicationContext& ctx) {
  if (reactive::CHECK || ctx.config_confirmed) {
    bool colored = ctx.config_confirmed;
    ctx.lcd.Fill_Rect(10, 10, 20, 20, colored ? 
        display::rgb_to_565(255, 0, 0) : display::rgb_to_565(0, 255, 0));
  }

  if (reactive::CHECK || ctx.playzone) {
    display::pintarCampo(ctx.lcd, ctx.playzone, ctx.starting_position);
  }

  if (reactive::CHECK || ctx.starting_position) {
    display::pintarSpawn(ctx.lcd, ctx.playzone, ctx.starting_position);
  }
}

// Context callbacks
void change_field(menus::ApplicationContext& ctx) {
  int pos = ctx.starting_position;
  pos %= 10;
  ++pos;

  ctx.playzone = !ctx.playzone;
  ctx.starting_position = pos;
  ctx.config_confirmed = false;
}

void change_starting_position(menus::ApplicationContext& ctx) {
  int pos = ctx.starting_position;

  if(pos != 0)
    ++pos;
  pos %= 10;
  ++pos;
  ctx.starting_position = pos;
  ctx.config_confirmed = false;
}

void plan(menus::ApplicationContext& ctx) {}

void validate_config(menus::ApplicationContext& ctx) {
  if (ctx.pendrive_plugged) {
    ctx.config_confirmed = true;
  }
}

menus::ContextMenuEntry strat_menus[4] = {
  menus::ContextMenuEntry("Campo", &change_field),
  menus::ContextMenuEntry("Start", &change_starting_position),
  menus::ContextMenuEntry("Plan"),
  menus::ContextMenuEntry(" Valid", &validate_config)
};


menus::ContextMenuEntry* menus::strat_ctx_menus() {
  return strat_menus;
}
