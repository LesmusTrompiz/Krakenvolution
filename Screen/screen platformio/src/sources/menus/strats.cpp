#include "display.hpp"
#include "menus.hpp"

void menus::strat_menu_start(menus::ApplicationContext& ctx) {
  ctx.playzone.trigger();
  ctx.starting_position.trigger();
  ctx.plan.trigger();
  ctx.config_confirmed.trigger();
  ctx.selected_bt.trigger();
}

constexpr int PLAN_TEXT_Y = 243;
display::SegmentedText plan_text("", 0, PLAN_TEXT_Y, 415, -1);

void menus::strat_menu_update(ApplicationContext& ctx) {
  int chekNext = ctx.starting_position;
  chekNext += 2;
  chekNext %= 10;

  if (reactive::CHECK || ctx.playzone) {
    display::pintarCampo(ctx.lcd, ctx.playzone, ctx.starting_position);
  }

  if (reactive::CHECK || ctx.starting_position) {
    display::pintarSpawnParking(ctx.lcd, ctx.playzone, ctx.starting_position, true, ctx.parking_position);
  }

  if (reactive::CHECK || ctx.parking_position) {
    display::pintarSpawnParking(ctx.lcd, ctx.playzone, ctx.parking_position, false, ctx.parking_position);

    if(chekNext == ctx.parking_position)
      display::pintarSpawnParking(ctx.lcd, ctx.playzone, ctx.starting_position, true, ctx.parking_position);
  }

  if (reactive::CHECK || ctx.selected_bt) {
    if (ctx.bt_list.size())
      plan_text = display::SegmentedText(ctx.bt_list[ctx.selected_bt].c_str(), 0, PLAN_TEXT_Y, 415, -1);
    ctx.lcd.Fill_Rect(0, 240, 415 - 30, 30, display::BLACK);
  }

  ctx.lcd.Set_Text_Size(3);
  ctx.lcd.Set_Text_colour(display::WHITE);
  plan_text.update(ctx.lcd);

  if (reactive::CHECK || ctx.config_confirmed) {
    ctx.lcd.Set_Draw_color(ctx.config_confirmed ? 
      display::rgb_to_565(0, 255, 0) :
      display::rgb_to_565(255, 0, 0)
    );
    ctx.lcd.Fill_Circle(415 - 15, PLAN_TEXT_Y + 25 / 2, 25 / 2);
  }
}

// Context callbacks
void change_field(menus::ApplicationContext& ctx) {
  int pos = ctx.starting_position;
  int park = ctx.parking_position;
  pos %= 10;
  park %= 10;
  ++pos;
  ++park;

  ctx.playzone = !ctx.playzone;
  ctx.starting_position = pos;
  ctx.parking_position = park;
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

void update_plan(menus::ApplicationContext& ctx) {
  if (ctx.bt_list.size()) {
    int index = ctx.selected_bt;
    ++index %= ctx.bt_list.size();
    ctx.selected_bt = index;
    ctx.config_confirmed = false;
  }
}

void change_parking(menus::ApplicationContext& ctx) {
  int park = ctx.parking_position;

  if(park != 0)
    ++park;
  park %= 10;
  ++park;
  ctx.parking_position = park;
  ctx.config_confirmed = false;
}

menus::ContextMenuEntry strat_menus[4] = {
  menus::ContextMenuEntry("Campo", &change_field),
  menus::ContextMenuEntry("Start", &change_starting_position),
  menus::ContextMenuEntry("Plan", &update_plan),
  menus::ContextMenuEntry("Park", &change_parking)
};


menus::ContextMenuEntry* menus::strat_ctx_menus() {
  return strat_menus;
}
