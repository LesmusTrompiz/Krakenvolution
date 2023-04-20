#include "display.hpp"
#include "menus.hpp"

display::SegmentedText txt("", 8, 8, 416, 233);
String errors = "";

        // mylcd.Fill_Rect(  8, 8, 416, 233, BLACK);
void menus::debug_menu_start(menus::ApplicationContext& ctx) {
  txt.reset();
}

void menus::debug_menu_update(ApplicationContext & ctx) {
  ctx.lcd.Set_Text_Mode(display::IGNORE_BG);
  ctx.lcd.Set_Text_colour(display::WHITE);
  ctx.lcd.Set_Text_Size(3);
  txt.update(ctx.lcd);
}

void menus::insertError(ApplicationContext& ctx, String error) {
  errors = "";
  ctx.errors[ctx.errorsPos] = error;
  ++ctx.errorsPos %= 10;

  for(int i = 0; i < 11; i++) {
    if(ctx.errors[i] != NULL)
      errors += ctx.errors[i] + "\n";
  }
  
  txt = display::SegmentedText(errors.c_str(), 8, 8, 416, 233);
}

menus::ContextMenuEntry debug_menus[4] = {
  menus::ContextMenuEntry("")
};

menus::ContextMenuEntry* menus::debug_ctx_menus() {
  return debug_menus;
}
