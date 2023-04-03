#include "serial_bridge/DummySerialNode.hpp"

using namespace std::chrono_literals;

// Utilidades
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

// Constructor
DummySerialBridgeNode::DummySerialBridgeNode(std::string port_name)
: Node("dummy_serial_bridge_node"){
  /*
    En el constructor se levantan los siguientes elementos:
      - Un timer que llama al ciclo de control con un periodo 500ms.
      - Un publisher para publicar la odometria el robot.
      - Un servidor de servicio que permite modificar el valor de la 
        odometria del robot.
      - Un servidor de acción que se encarga de realizar las llamadas
        al RMI o en este caso a las funciones de simulación.
  */

  using namespace std::placeholders;

  // Topic donde se publica el valor de la odometría.
  publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("robot_odom", 10);

  // Servicio que permite modificar el valor de la odometria.
  reset_service = this->create_service<uahrk_navigation_msgs::srv::SetPose2d>("set_pose", 
      std::bind(&DummySerialBridgeNode::set_pose, this, _1 ,_2));
  
  // servidor de acción que se encarga de realizar las llamadas
  // al RMI o en este caso a las funciones de simulación.
  order_server =  rclcpp_action::create_server<serial_bridge_actions::action::Order>(
    this,
    "serial_bridge_server",
    std::bind(&DummySerialBridgeNode::handle_goal,     this, _1, _2),
    std::bind(&DummySerialBridgeNode::handle_cancel,   this, _1),
    std::bind(&DummySerialBridgeNode::handle_accepted, this, _1));

  // Timer que llama al ciclo de control con un periodo 500ms.
  timer_ = this->create_wall_timer(
    500ms, std::bind(&DummySerialBridgeNode::control_cycle, this));
}

// Destructor
DummySerialBridgeNode::~DummySerialBridgeNode(){}


// Ciclo de control
void DummySerialBridgeNode::control_cycle(){
  /*
    El ciclo de control de este nodo se encarga
    de:
      - Llamar a las funciones de simulación.
      - Eliminar las funciones de simulación que 
        ya hallan acabado.
      - Publicar la odometria simulada.
  */
  
  // Variables auxiliares.
  int n = 0;                                                    // n sirve para llevar la cuenta en el bucle
  std::vector<int> succeed_fns;                                 // almacena el indice de las funciones que ya han acabado
  
  // Recorremos todas las funciones de simulación
  // y en caso de que hallan acabado almacenamos 
  // su posición dentro del array. Para más tarde 
  // borrarlo.
  for(auto &f : tick_functions){
    auto succeed = std::get<0>(f)(odom, std::get<1>(f));
    if(succeed) succeed_fns.push_back(n);
    n++;
  }

  // Recorre el vector que contiene los indices de las
  // funciones de simulación que han termindado,
  // eliminando en cada iteración la funcion asociada
  // al indice. El vector lo recorre de atás hacia 
  // adelante para que no le afecte el desplazamiento
  // originidado al eliminar funciones.
  for (auto it = succeed_fns.rbegin(); it != succeed_fns.rend(); ++it){
    tick_functions.erase(tick_functions.begin()+*it);
  }

  // Publicar la odometria
  publisher_->publish(Pose2dtoPose(odom));
}


// Callbacks del servidor de acción:
rclcpp_action::GoalResponse DummySerialBridgeNode::handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const Order::Goal> goal){
  /*
    Función que se llama cuando nos envian una petición por 
    el servidor de acción. Se aceptan todas las peticiones.
  */

  // Debug Info
  RCLCPP_INFO(this->get_logger(), "Order received");

  // Supress warnings of not beeing used
  (void)uuid;

  // Accept all request
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse  DummySerialBridgeNode::handle_cancel(
  const std::shared_ptr<GoalOrder> goal_handle){
  /*
    Función que se llama cuando nos envian una petición para 
    cancelar un petición pasada. 

    Como en el simulador no tiene sentido cancelar peticiones,
    se hace un reject de la petición de cancel, 
  */
 
  // Supress warnings of not beeing used
  (void)goal_handle;
 
  return rclcpp_action::CancelResponse::REJECT;
}

void DummySerialBridgeNode::handle_accepted(const std::shared_ptr<GoalOrder> goal_handle){
  /*
    Esta función es llamada cuando se acepta una petición del servidor de acción.
    Esta función busca si existe una función que pueda simular el comportamiento
    del RMI de la acción pedida. En caso de que pueda simularla crea una lambda
    que se llamará en el control cycle del nodo. 
    
    En caso de que no pueda o ocurra algún error durante su creación, cancela
    el goal y crea un log.
  */

  // Almacena el objetivo
  const auto goal = goal_handle->get_goal();

  // Crea un log con la petición que se ha recivido.
  RCLCPP_INFO(this->get_logger(), "Handling order: Id %s, Arg %d", goal->id.c_str(), goal->arg);
  
  try{
    // Consulta si existe una función que
    // pueda simular la acción pedida.
    auto sim_fn = simulate.find(goal->id);
    
    // Si no existe, cancela la petición
    if (sim_fn == simulate.end()){
      auto result   = std::make_shared<Order::Result>();
      RCLCPP_ERROR(this->get_logger(), "Error : Id %s is not an RMI function", goal->id.c_str());
      goal_handle->canceled(result);
      return;
    }

    // Si existe:
    // Crea una lambda que simula la acción, y se encargue 
    // de indicarle al cliente que la petición ha finalizado.
    auto f = [request =std::move(goal_handle), sim_fn](Pose2d &odom, int16_t &arg) { 
      auto result   = std::make_shared<Order::Result>();
      
      // Simular la acción
      auto succeed  = sim_fn->second(odom,arg);
      
      // Si acaba notificar al cliente
      if (succeed) request->succeed(result);
      return succeed;
    };

    // Almacena la función de simulación en un vector que
    // será utilizado en el ciclo de control.
    tick_functions.emplace_back(std::make_tuple(f,goal->arg));
    return;
  }

  // Si ocurre cualquier error no previsto por el código simplemente cancela la petición.
  catch(const std::exception& e)
  {
    auto result   = std::make_shared<Order::Result>();
    RCLCPP_ERROR(this->get_logger(), "Error : simulating action %s", e.what());
    goal_handle->canceled(result);
  }
}


// CB del servicio:
void DummySerialBridgeNode::set_pose(
    const std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Request> request,
    std::shared_ptr<uahrk_navigation_msgs::srv::SetPose2d::Response> response){
  /*
    Esta función se llama cuando recive una petición por el servicio
    para actualizar la odometría. 
    
    Simplemente asigna el valor de la petición al de la odometría.
  */
  odom.x = request->x;
  odom.y = request->y;
  odom.a = request->a;

  RCLCPP_INFO(this->get_logger(), "Setting pose by service to: x: %f y: %f a: %f", request->x
  , request->y, request->a);
}

