#include "demo_nova_sanctum/air_collector_tank.h"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

AirPublisher::AirPublisher()
: Node("air_publisher"),
  flow_rate_(this->declare_parameter<double>("flow_rate", 28.0)),
  co2_intake_(this->declare_parameter<double>("co2_intake", 1.04)),
  crew_onboard_(this->declare_parameter<int>("crew_onboard", 4)),
  cabin_pressure_(this->declare_parameter<double>("cabin_pressure", 14.7)),
  temperature_cutoff_(this->declare_parameter<double>("temperature_cutoff", 450.0)),
  max_crew_limit_(this->declare_parameter<int>("max_crew_limit", 6)),
  power_consumption_(this->declare_parameter<double>("power_consumption", 1.0)),
  system_name_(this->declare_parameter<std::string>("system_name", "demo_nova_sanctum")),
  mode_of_operation_(this->declare_parameter<std::string>("mode_of_operation", "standby")),
  tank_capacity_(this->declare_parameter<double>("tank_capacity", 1000.0)) { 

  // Air mixture parameters
  co2_mass_ = 0.0;
  moisture_content_ = 0.0;
  contaminants_ = 0.0;
  dew_point_ = 10.0; 
  total_air_mass_ = 0.0;

  // ROS publishers and clients
  unpure_air_publisher_ = this->create_publisher<demo_nova_sanctum::msg::AirData>("/unpure_air", 10);
  desiccant_server_client_ = this->create_client<std_srvs::srv::Trigger>("/desiccant_server");
  adsorbent_server_client_ = this->create_client<std_srvs::srv/Trigger>("/adsorbent_server");

  // ROS timer
  timer_ = this->create_wall_timer(1s, std::bind(&AirPublisher::timer_callback, this));
}

void AirPublisher::timer_callback() {
  double k = cabin_pressure_ / temperature_cutoff_;
  double h_ratio = 0.05; 
  double c_ratio = 0.02; 

  // Update temperature cutoff for simulation purposes
  temperature_cutoff_ = 30.0;

  // Calculate CO2 mass increment
  co2_mass_ += flow_rate_ * co2_intake_ * crew_onboard_ * k;

  // Calculate moisture increment
  moisture_content_ += flow_rate_ * h_ratio * crew_onboard_;

  // Calculate contaminants increment
  contaminants_ += flow_rate_ * c_ratio * crew_onboard_;

  // Update dew point based on temperature and moisture content
  dew_point_ = 10.0 + 0.1 * moisture_content_;

  // Calculate total air mass
  total_air_mass_ = co2_mass_ + moisture_content_ + contaminants_;

  // Create and publish the air data message
  auto message = demo_nova_sanctum::msg::AirData();
  message.co2_mass = co2_mass_;
  message.moisture_content = moisture_content_;
  message.contaminants = contaminants_;
  message.temperature = temperature_cutoff_;
  message.dew_point = dew_point_;

  RCLCPP_INFO(this->get_logger(), "Publishing air data: CO2: %.2f g, Moisture: %.2f %%, Contaminants: %.2f %%",
              co2_mass_, moisture_content_, contaminants_);

  unpure_air_publisher_->publish(message);

 
  if (total_air_mass_ >= tank_capacity_) {
    RCLCPP_WARN(this->get_logger(), "Tank capacity reached! Triggering servers and opening valve...");
    trigger_server("/desiccant_server", desiccant_server_client_);
    trigger_server("/adsorbent_server", adsorbent_server_client_);
    open_valve();
  }
}

void AirPublisher::trigger_server(const std::string &server_name,
                                  const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client) {
  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(this->get_logger(), "Service %s is not available!", server_name.c_str());
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  try {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Triggered server %s successfully.", server_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to trigger server %s: %s", server_name.c_str(), response->message.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while calling service %s: %s", server_name.c_str(), e.what());
  }
}

void AirPublisher::open_valve() {
  RCLCPP_INFO(this->get_logger(), "Opening the valve to release air...");
  // Simulate valve opening
  total_air_mass_ = 0.0; // Reset the total air mass after releasing air
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AirPublisher>());
  rclcpp::shutdown();
  return 0;
}
