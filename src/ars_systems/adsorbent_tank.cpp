#include "demo_nova_sanctum/adsorbent_tank.h"

AdsorbentBed::AdsorbentBed()
: Node("adsorbent_bed"),
  co2_removal_efficiency_(this->declare_parameter<double>("co2_removal_efficiency", 0.90)), // 90% removal
  co2_to_space_ratio_(this->declare_parameter<double>("co2_to_space_ratio", 0.50)), // 50% sent to space
  is_active_(false) {

  // Subscriber to the /removed_moisture topic
  air_data_subscriber_ = this->create_subscription<demo_nova_sanctum::msg::AirData>(
    "/removed_moisture", 10, std::bind(&AdsorbentBed::air_data_callback, this, std::placeholders::_1));

  // Publisher for the /processed_co2 topic
  processed_co2_publisher_ = this->create_publisher<demo_nova_sanctum::msg::AirData>("/processed_co2", 10);

  // Service to handle trigger requests
  trigger_server_ = this->create_service<std_srvs::srv::Trigger>(
    "/adsorbent_server", std::bind(&AdsorbentBed::trigger_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));
}

void AdsorbentBed::air_data_callback(const demo_nova_sanctum::msg::AirData::SharedPtr msg) {
  if (!is_active_) {
    RCLCPP_WARN(this->get_logger(), "Adsorbent bed is not active. Ignoring incoming data.");
    return;
  }

  // Calculate CO2 reduction
  double co2_removed = msg->co2_mass * co2_removal_efficiency_;
  double co2_to_space = co2_removed * co2_to_space_ratio_;
  double co2_for_processing = co2_removed - co2_to_space;

  // Publish the CO2 for further processing
  auto new_msg = demo_nova_sanctum::msg::AirData();
  new_msg.co2_mass = co2_for_processing;
  new_msg.moisture_content = 0.0; // No moisture should remain
  new_msg.contaminants = 0.0;    // No contaminants should remain
  
  new_msg.temperature = msg->temperature;
  new_msg.dew_point = msg->dew_point;

  RCLCPP_INFO(this->get_logger(),
              "Processed CO2: %.2f g for further use, %.2f g sent to space.",
              co2_for_processing, co2_to_space);

  processed_co2_publisher_->publish(new_msg);
}

void AdsorbentBed::trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                    std_srvs::srv::Trigger::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "Received trigger to activate the adsorbent bed.");
  is_active_ = true;
  response->success = true;
  response->message = "Adsorbent bed activated successfully.";
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdsorbentBed>());
  rclcpp::shutdown();
  return 0;
}
