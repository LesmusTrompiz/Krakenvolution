#include "serial_bridge/SerialBridgeNode.hpp"




SerialBridgeNode::SerialBridgeNode(std::string port_name)
: Node("serial_bridge_node"){
  
  // Topic donde se publica el valor de la odometría.
  publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_odom", 10);

  // Servicio que permite modificar el valor de la odometria, desde otros nodos.
  // Es util llamarlo cuando se inicializa el spawn y ese tipo de cosas.
  reset_service = this->create_service<uahrk_navigation_msgs::srv::SetPose2d>("set_pose", 
      std::bind(&SerialBridgeNode::set_pose, this, _1 ,_2));
  
  // servidor de acción que se encarga de recibir las peticiones que se quieren
  // hacer a través del RMI. Utiliza un mensaje de tipo serial_bridge_actions::action::Orderr
  order_server =  rclcpp_action::create_server<serial_bridge_actions::action::Order>(
    this,
    "serial_bridge_server",
    std::bind(&SerialBridgeNode::handle_goal,     this, _1, _2),
    std::bind(&SerialBridgeNode::handle_cancel,   this, _1),
    std::bind(&SerialBridgeNode::handle_accepted, this, _1));

  //** TODO: Entiendo que por aqui se configura el protocolo....

  
  // Timer que ejecuta la función control_cycle cada 500ms 
  // A lo mejor te interesa crear tu propio hilo...
  timer_ = this->create_wall_timer(
    500ms, std::bind(&SerialBridgeNode::control_cycle, this));
}

SerialBridgeNode::~SerialBridgeNode(){}


// Ciclo de control
void SerialBridgeNode::control_cycle(){
  /*
    El ciclo de control de este nodo se encarga
    de:
      - Leer los mensajes del protocolo.
      - Avisar de que la arduino ha acabado
        de hacer la acción correspondiente.
      - Publicar la odometria.
  */
  
  // TODO: Leer del protocolo

    // TODO: En caso de que haya acabado alguna acción avisar de que ha acabado
    // Para ello extraer de alguna estructura de la clase el goal handle y
    // llamar al metodo succed o al fail
  

    // TODO: Actualizar la odometría


  // Publicar la odometria
  publisher_->publish(Pose2dtoPose(odom));
}


// CB del servicio:
void SerialBridgeNode::set_pose(
    const std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Request> request,
    std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Response> response){
  /*
    Esta función es llamad cuando recive una petición por el servicio
    para actualizar la odometría. 
    
    Simplemente asigna el valor de la petición al de la odometría.
  */
  odom.x = request->x;
  odom.y = request->y;
  odom.a = request->a;

  RCLCPP_INFO(this->get_logger(), "Setting pose by service to: x: %f y: %f a: %f", request->x
  , request->y, request->a);
}


rclcpp_action::GoalResponse SerialBridgeNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Order::Goal> goal){
  /*
    Esta función es llamada cuando el cliente realiza
    alguna petición, desde aqui se pueden aceptar o 
    denegar las peticiones del cliente. 
    
    En nuestro caso no tiene mucho sentido denegar 
    ninguna petición, asi que aceptamos y pa alante
  */

  // Accept all request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse  SerialBridgeNode::handle_cancel(
  const std::shared_ptr<GoalOrder> goal_handle){
  /*
    Esta función es llamada cuando el cliente quiere
    cancelar alguna petición, se puede responder
    indicando si ha sido posible cancelar la acción 
    o no.
    
    A pesar de que tenga mucho sentido cancelar alguna
    acción en el RMI, ningún nodo de la arquitectura lo
    va a pedir a si que no hace falta implementarlo.
  */

  // Reject all  Cancel request
  return rclcpp_action::CancelResponse::REJECT;
}

void SerialBridgeNode::handle_accepted(const 
  std::shared_ptr<GoalOrder> goal_handle)
{
  /*
    Esta función es llamada cuando la petición
    es aceptada. El argumento contiene la
    petición que se ha realizado, por tanto
    en esta función se encuentra en el kit 
    de la cuestión.
    
    Esta función deberia de retornar rápido
    por lo que deberia de hacer es:
      - Hacer la llamada RMI o meter la llamada en
        el la cola de mensajes 
      - Almacenar el goal_handle para más tarde 
        indicar si la acción ha sido completada
        o no.

    
    El codigo para indicar que una petición ha sido acabada
    correctamente se resume en;

      // Crear una estructura result
      auto result   = std::make_shared<Order::Result>();
      // Llamar a la función succed.
      goal_handle->succeed(result);
  */

  const auto goal = goal_handle->get_goal();
  RCLCPP_INFO(this->get_logger(), "RMI ORDER: Id %s, Arg %d", goal->id.c_str(), goal->arg);
  
  // TODO Enviar la instrucción al protocolo
  

  // Almacenar el goal handle en alguna estructura para
  // indicar que ha terminado 

  return;
}


