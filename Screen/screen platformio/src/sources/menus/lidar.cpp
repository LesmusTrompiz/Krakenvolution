#include "menus.hpp"

display::SegmentedText txt_turn_off("Seleccione\napagar para\ndetener el\nlidar.\nEstado del\nlidar: ON", 8, 8, 416, 233);
display::SegmentedText txt_turn_on("Seleccione\nencender para\narrancar el\nlidar.\nEstado del\nlidar: OFF", 8, 8, 416, 233);

void menus::lidar_menu_start(menus::ApplicationContext& ctx) {
  txt_turn_off.reset();
  txt_turn_on.reset();
}

void menus::lidar_menu_update(menus::ApplicationContext& ctx) {
  ctx.lcd.Set_Text_Size(5);

  if(ctx.lidar) {
    txt_turn_off.update(ctx.lcd);
  } else {
    txt_turn_on.update(ctx.lcd);
  }
}

void turn_lidar(menus::ApplicationContext& ctx) {
  ctx.lidar = !ctx.lidar;

  if(ctx.lidar)
    Serial.write("{\"command\":{\"name\":turn_lidar,\"arg\":true}}");
  else
    Serial.write("{\"command\":{\"name\":turn_lidar,\"arg\":false}}");

  txt_turn_off.reset();
  txt_turn_on.reset();
  ctx.lcd.Fill_Rect(  0, 0, 423, 272, 0x0000);
}

menus::ContextMenuEntry lidar_menus[4] = {
  menus::ContextMenuEntry("Turn", &turn_lidar)
};

menus::ContextMenuEntry* menus::lidar_ctx_menus() {
  return lidar_menus;
}
