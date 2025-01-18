#ifndef demo_nova_sanctum_AIR_PUBLISHER_HPP_
#define demo_nova_sanctum_AIR_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "std_srvs/srv/trigger.hpp"

class AirPublisher : public rclcpp::Node {
public:
  AirPublisher();

private:
  void timer_callback();
  void trigger_server(const std::string &server_name, const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client);
  void open_valve();

  // Parameters
  double flow_rate_;             // Flow rate in SCFM
  double co2_intake_;            // CO2 intake in mmHg
  int crew_onboard_;             // Number of astronauts onboard
  double cabin_pressure_;        // Cabin pressure in PSI
  double temperature_cutoff_;    // Temperature cutoff in Celsius
  int max_crew_limit_;           // Maximum crew capacity
  double power_consumption_;     // Power consumption in kW
  double tank_capacity_;         // Maximum tank capacity for air mass in grams
  std::string system_name_;      // System name
  std::string mode_of_operation_; // Mode of operation (standby/active/emergency)

  // Air mixture parameters
  double co2_mass_;              // Current CO2 mass in grams
  double moisture_content_;      // Moisture content in percentage
  double contaminants_;          // Contaminants in percentage
  double dew_point_;             // Dew Point in Celsius
  double total_air_mass_;        // Total air mass in grams

  // ROS publishers, clients, and timers
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr unpure_air_publisher_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr desiccant_server_client_;
  
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // demo_nova_sanctum_AIR_PUBLISHER_HPP_
