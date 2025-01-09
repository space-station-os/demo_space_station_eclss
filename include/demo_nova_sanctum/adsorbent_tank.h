#ifndef demo_nova_sanctum_ADSORBENT_BED_HPP_
#define demo_nova_sanctum_ADSORBENT_BED_HPP_

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "std_srvs/srv/trigger.hpp"

class AdsorbentBed : public rclcpp::Node {
public:
  AdsorbentBed();

private:
  void air_data_callback(const demo_nova_sanctum::msg::AirData::SharedPtr msg);
  void trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        std_srvs::srv::Trigger::Response::SharedPtr response);

  // Parameters
  double co2_removal_efficiency_; // Percentage of CO2 removed
  double co2_to_space_ratio_;     // Ratio of CO2 sent to space

  // ROS subscribers, publishers, and servers
  rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr air_data_subscriber_;
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr processed_co2_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_;

  // State variables
  bool is_active_; // Whether the adsorbent bed server is active
};

#endif // demo_nova_sanctum_ADSORBENT_BED_HPP_
