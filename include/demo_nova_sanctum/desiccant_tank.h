#ifndef demo_nova_sanctum_DESICCANT_SERVER_HPP_
#define demo_nova_sanctum_DESICCANT_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "std_srvs/srv/trigger.hpp"

class DesiccantServer : public rclcpp::Node {
public:
  DesiccantServer();

private:
  void air_data_callback(const demo_nova_sanctum::msg::AirData::SharedPtr msg);
  void trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);

  // Parameters
  double moisture_removal_rate_;    // Percentage of moisture removed
  double contaminant_removal_rate_; // Percentage of contaminants removed

  // ROS subscribers, publishers, and servers
  rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr air_data_subscriber_;
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr removed_moisture_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_;

  // State variables
  bool is_active_; // Whether the desiccant server is active
};

#endif // demo_nova_sanctum_DESICCANT_SERVER_HPP_
