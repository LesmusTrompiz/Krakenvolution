#include "serial_bridge/SerialBridgeNode.hpp"
#include "serial_bridge/serial_port.hpp"
#include <thread>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

uahruart::parser::Protocol protocol;

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

  // Configuración básica del protocolo
    // Abrir el puerto serie
  auto port = open_serial(port_name.c_str());
  if (port == -1) {
    RCLCPP_ERROR(this->get_logger(), "Could not open port %s", port_name.c_str());
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "Opened serial port %s", port_name.c_str());

    // Configure thread for reading port
  read_thread = std::thread([&, port]() {
    string str;
    while (rclcpp::ok()) {
        char c;
        read(port, &c, 1);
        if (c == '\0' || c == '\n') {
            std::cout << "Read string: " << str << '\n';
            protocol.receive(str);
            str = "";
        } else {
            str += c;

        }
      }
  });

  // read_thread.detach(); // Run thread in daemon mode

  protocol.on_write([port](const char* buff) {
      std::cout << "Writing: " << buff << '\n';
    size_t ammount = strlen(buff);
    write(port, buff, ammount);
    write(port, "\n\0", 2); // Flush 
  });

  protocol.on_type(uahruart::IDs::RPC_RESPONSE, std::function<void(const uahruart::messages::RPCResponse&)>{[&](auto msg) {
    std::cout << "Response of type: " << msg.ret.to_underlying() << '\n';
    if (m_pending_handles.size()) {
      std::cout << "Added handle to list\n";
      auto handle = m_pending_handles.front();
      m_pending_handles.pop_front();

      m_handles[msg.ret.to_underlying()] = handle;
    }
  }});

  protocol.on_type(uahruart::IDs::ACTION_FINISHED, std::function<void(const uahruart::messages::ActionFinished&)>{[&](auto msg) {
    std::cout << "You dun goofed\n";
    std::cout << "Finished action of type: " << msg.action.to_underlying() << '\n';
    if (msg.action.to_underlying() == uahruart::messages::ActionFinished::TRACTION) {
      m_pending_last_odom = true;
    }
    else if (m_handles[msg.action.to_underlying()]) {
      auto result   = std::make_shared<Order::Result>();
      m_handles[msg.action.to_underlying()]->succeed(result);
      m_handles[msg.action.to_underlying()] = nullptr;
    }
  }});

  protocol.on_type(uahruart::IDs::ODOMETRY, std::function<void(const uahruart::messages::Odometry&)>{[&](auto msg) {
    auto pose = Pose2d(
      static_cast<float>(msg.x.to_underlying()),
      static_cast<float>(msg.y.to_underlying()),
      static_cast<float>(msg.o.to_underlying())
    );
    publisher_->publish(Pose2dtoPose(odom));
    if (m_handles[uahruart::messages::ActionFinished::TRACTION]) {
      auto result   = std::make_shared<Order::Result>();
      m_handles[uahruart::messages::ActionFinished::TRACTION]->succeed(result);
      m_handles[uahruart::messages::ActionFinished::TRACTION] = nullptr;
    }
  }});
  
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
  
    // TODO: En caso de que haya acabado alguna acción avisar de que ha acabado
    // Para ello extraer de alguna estructura de la clase el goal handle y
    // llamar al metodo succed o al fail
  

  // // Publicar la odometria
  // publisher_->publish(Pose2dtoPose(odom));
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
  m_pending_handles.push_back(goal_handle);
  
  // TODO Enviar la instrucción al protocolo
  
  // goal->succeed(0);
  uahruart::messages::RPCCall call;
  call.function_hash = uahruart::utils::hash_string(goal->id.c_str()) ^ uahruart::utils::hash_string(goal->device.c_str());
  call.call_uuid = 0;
  call.arg = goal->arg;
  while(!protocol.send(call)) {usleep(1000);}


  // Almacenar el goal handle en alguna estructura para
  // indicar que ha terminado 

  return;
}


