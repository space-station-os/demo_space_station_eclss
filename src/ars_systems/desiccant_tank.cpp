#include "demo_nova_sanctum/desiccant_tank.h"
#include <mutex> // Include for mutex

DesiccantServer::DesiccantServer()
: Node("desiccant_server"),
  moisture_removal_rate_(this->declare_parameter<double>("moisture_removal_rate", 0.95)), // 95% total removal
  contaminant_removal_rate_(this->declare_parameter<double>("contaminant_removal_rate", 0.90)), // 90% total removal
  is_active_(false) {

  // Subscriber to the /unpure_air topic
  air_data_subscriber_ = this->create_subscription<demo_nova_sanctum::msg::AirData>(
    "/unpure_air", 10, std::bind(&DesiccantServer::air_data_callback, this, std::placeholders::_1));

  // Publisher for the /removed_moisture topic
  removed_moisture_publisher_ = this->create_publisher<demo_nova_sanctum::msg::AirData>("/removed_moisture", 10);

  // Service to handle trigger requests
  trigger_server_ = this->create_service<std_srvs::srv::Trigger>(
    "/desiccant_server", std::bind(&DesiccantServer::trigger_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));
}

void DesiccantServer::air_data_callback(const demo_nova_sanctum::msg::AirData::SharedPtr msg) {
  
  co2_ = msg->co2_mass;
  moisture_content_ = msg->moisture_content;
  contaminants_ = msg->contaminants;
  temperature_ = msg->temperature;
  dew_point_ = msg->dew_point;

  RCLCPP_INFO(this->get_logger(), "Received air data: CO2: %.2f g, Moisture: %.2f %%, Contaminants: %.2f %%",
              co2_, moisture_content_, contaminants_);
}

void DesiccantServer::process_air_data() {
  
  // Gradual reduction
  double moisture_decrement = (moisture_content_ * moisture_removal_rate_) * 0.1; // 10% per step
  double contaminants_decrement = (contaminants_ * contaminant_removal_rate_) * 0.1; // 10% per step

  moisture_content_ -= moisture_decrement;
  contaminants_ -= contaminants_decrement;

  // Ensure no negative values
  if (moisture_content_ < 0.0) {
    moisture_content_ = 0.0;
  }
  if (contaminants_ < 0.0) {
    contaminants_ = 0.0;
  }

  // Prepare the processed message
  demo_nova_sanctum::msg::AirData processed_msg;
  processed_msg.co2_mass = co2_; // CO2 remains unchanged in desiccant bed
  processed_msg.moisture_content = moisture_content_;
  processed_msg.contaminants = contaminants_;
  processed_msg.temperature = temperature_;
  processed_msg.dew_point = dew_point_;

  RCLCPP_INFO(this->get_logger(),
              "Processed air data: CO2: %.2f g, Remaining Moisture: %.2f %%, Remaining Contaminants: %.2f %%",
              processed_msg.co2_mass, processed_msg.moisture_content, processed_msg.contaminants);

  // Publish the processed data
  removed_moisture_publisher_->publish(processed_msg);
}

void DesiccantServer::trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                       const std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (!is_active_) {
    is_active_ = true;
    response->success = true;
    response->message = "Desiccant bed activated successfully.";

    RCLCPP_INFO(this->get_logger(), "Desiccant bed activated. Starting timer...");
    // Dynamically create the timer
    timer_ = this->create_wall_timer(500ms, std::bind(&DesiccantServer::process_air_data, this));
  } else {
    response->success = false;
    response->message = "Desiccant bed is already active.";
    RCLCPP_INFO(this->get_logger(), "Desiccant bed is already active.");
  }
}

// void DesiccantServer::deactivate() {
//   if (is_active_) {
//     is_active_ = false;
//     RCLCPP_INFO(this->get_logger(), "Deactivating desiccant bed and stopping timer...");
//     timer_.reset(); // Stop the timer
//   }
// }

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DesiccantServer>();

  rclcpp::spin(node);

  // Ensure proper cleanup
//   node->deactivate();
  rclcpp::shutdown();
  return 0;
}
