#include <chrono>
#include "../include/ekf_krakens.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

/**
 * @todo: Lo suyo sería que la clase del nodo de localización 
 * este separada del main. Como la biblioteca del filtro.
*/

class LocalizationNode : public rclcpp::Node
{

  public:
    LocalizationNode()
    : Node("localization_node")
    {
      //Matrices EKF
      A.setIdentity();
      B.setIdentity();
      C << 1, 1, 1;
      for(int i=0; i<3;++i){Q.diagonal()[i]=0.05;}
      for(int i=0; i<3;++i){R.diagonal()[i]=5;}
      P << .1, .1, .1, 10, 10, 10, 100, 100, 100;

      // Publicación cada 100ms
      pose_publisher  = this->create_publisher<geometry_msgs::msg::Pose>("uahrk_localization_pose", 10);
      timer_pub       = this->create_wall_timer(100ms, std::bind(&LocalizationNode::pose_pub_callback, this));
    }
  private:
    void pose_pub_callback()
    {
      auto final_pose = geometry_msgs::msg::Pose();

      final_pose.position.x      = 1.0;
      final_pose.position.y      = 2.0;
      final_pose.position.z      = 3.0;

      printf("X: %f \t Y: %f \t Th: %f \n", final_pose.position.x, final_pose.position.y, final_pose.position.z);

      pose_publisher->publish(final_pose);

    }

    rclcpp::TimerBase::SharedPtr timer_pub;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher;

    // Matrices del EKF
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C; 
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    // EKF
    EKFilter PoseFilter;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
