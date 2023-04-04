#include "serial_bridge/geometry_utils.hpp"

geometry_msgs::msg::Pose Pose2dtoPose(const Pose2d &p2d){
  /*
    Esta función transforma un struct de tipo Pose2D a
    un mensaje de geometry_msgs::msg::Pose de ROS.
  */
  geometry_msgs::msg::Pose p;

  // La transformación entre coordenadas x e y es directa
  p.position.x = p2d.x;
  p.position.y = p2d.y;

  // En cambio hay que transformar el ángulo en grados a quaternios
  tf2::Quaternion quat_tf;
  quat_tf.setRPY(0.0, 0.0, DEG2RAD(p2d.a));
  tf2::convert(quat_tf.normalize(), p.orientation);
  return p;
}