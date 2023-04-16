#include "display.hpp"
#include "menus.hpp"

display::SegmentedText txt("Lorem ipsum dolor sit amet, officia excepteur ex fugiat reprehenderit enim labore culpa sint ad nisi Lorem pariatur mollit ex esse exercitation amet. Nisi anim cupidatat excepteur officia. Reprehenderit nostrud nostrud ipsum Lorem est aliquip amet voluptate voluptate dolor minim nulla est proident. Nostrud officia pariatur ut officia. Sit irure elit esse ea nulla sunt ex occaecat reprehenderit commodo officia dolor Lorem duis laboris cupidatat officia voluptate. Culpa proident adipisicing id nulla nisi laboris ex in Lorem sunt duis officia eiusmod. Aliqua reprehenderit commodo ex non excepteur duis sunt velit enim. Voluptate laboris sint cupidatat ullamco ut ea consectetur et est culpa et culpa duis.",
                           8, 8, 416, 233);

        // mylcd.Fill_Rect(  8, 8, 416, 233, BLACK);
void menus::debug_menu_start(menus::ApplicationContext& ctx) {
  txt.reset();
}

void menus::debug_menu_update(ApplicationContext & ctx) {
  ctx.lcd.Set_Text_Mode(display::IGNORE_BG);
  txt.update(ctx.lcd);
}


menus::ContextMenuEntry debug_menus[4] = {
  menus::ContextMenuEntry("")
};

menus::ContextMenuEntry* menus::debug_ctx_menus() {
  return debug_menus;
}
