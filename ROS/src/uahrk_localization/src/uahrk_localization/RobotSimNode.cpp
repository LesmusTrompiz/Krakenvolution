#include "uahrk_localization/RobotSimNode.hpp"

RobotSimNode::RobotSimNode(const char *node_name, const char *topic_name, int it_sim, float referencia[3])
  : Node(node_name)
{
  // Los datos de la simulación se llevan a un archivo para poder visualizarlos más tarde...
  ofstream robot_data;
  robot_data.open("odom.txt");
  // SIMULACIÓN
  // Variables necesarias para la simulación
  float x_error   = 0;
  float y_error   = 0;
  float x_error_r = 0;
  float y_error_r = 0;
  float phi_error = 0;
  // Referencias
  x_ref   = referencia[0];
  y_ref   = referencia[1];
  phi_ref = referencia[2];
  // Insertamos la posición inicial
  x.push_back(x_0);
  y.push_back(y_0);
  phi.push_back(z_0);
  // Bucle de simulación
  for(int j=0;j<it_sim;j++)
  {
    // Guardamos los datos
    robot_data << x.back() << "\t" << y.back() << "\t" << phi.back() << std::endl; 
    // Errores de control
    x_error     = x_ref - x.back();
    y_error     = y_ref - y.back();
    x_error_r   = cos(phi.back())*x_error + sin(phi.back())*y_error;
    phi_error   = phi_ref - phi.back();
    // Ley de control
    vu.push_back(Kx*x_error_r);
    wu.push_back(Ky*phi_error);
    // Ahora sí, simulamos la planta
    x.push_back(x.back()+Ts*vu.back()*cos(phi.back()));
    y.push_back(y.back()+Ts*vu.back()*sin(phi.back()));
    phi.push_back(phi.back()+Ts*wu.back());
  }
  // Cerramos el archivo para evitar problemas...
  robot_data.close();
  // PUBLISHER
  // Publicación cada 100ms
  robot_pub             = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 10);
  timer_pub             = this->create_wall_timer(100ms, std::bind(&RobotSimNode::robot_callback, this));
}

void RobotSimNode::robot_callback()
{
  // "Parseamos" la información...
  auto actual_pose = geometry_msgs::msg::PoseStamped();
  if(!x.empty() && !y.empty() && !phi.empty())
  {
    actual_pose.pose.position.x   = x.front();    x.erase(x.begin());
    actual_pose.pose.position.y   = y.front();    y.erase(y.begin());
    actual_pose.pose.position.z   = phi.front();  phi.erase(phi.begin());
    // Info:
      RCLCPP_INFO(this->get_logger(), "Publicando simulación...");
    // Publicamos la información
    robot_pub->publish(actual_pose);
  }
  else
    return;
}