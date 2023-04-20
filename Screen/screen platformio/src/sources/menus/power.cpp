#include "menus.hpp"

display::SegmentedText txt_turn("Seleccione\napagar o\nreiniciar\nel robot", 8, 8, 416, 233);

void menus::power_menu_start(menus::ApplicationContext& ctx) {
  txt_turn.reset();
}

void menus::power_menu_update(ApplicationContext& ctx) {
  ctx.lcd.Set_Text_Size(6);
  txt_turn.update(ctx.lcd);
}

void turn_robot(menus::ApplicationContext& ctx) {
    Serial.write("{\"command\":{\"name\":turn_robot,\"arg\":true}}");
}

void reboot_robot(menus::ApplicationContext& ctx) {
    Serial.write("{\"command\":{\"name\":reboot_robot,\"arg\":true}}");
}

menus::ContextMenuEntry power_menus[4] = {
  menus::ContextMenuEntry("Turn", &turn_robot),
  menus::ContextMenuEntry("Reset", &reboot_robot)
};

menus::ContextMenuEntry* menus::power_ctx_menus() {
  return power_menus;
}
