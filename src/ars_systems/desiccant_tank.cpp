#include "demo_nova_sanctum/desiccant_server.hpp"

DesiccantServer::DesiccantServer()
: Node("desiccant_server"),
  moisture_removal_rate_(this->declare_parameter<double>("moisture_removal_rate", 0.95)), // 95% removal
  contaminant_removal_rate_(this->declare_parameter<double>("contaminant_removal_rate", 0.90)), // 90% removal
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
  if (!is_active_) {
    RCLCPP_WARN(this->get_logger(), "Desiccant server is not active. Ignoring incoming data.");
    return;
  }

  // Remove moisture and contaminants from the air
  double remaining_co2_mass = msg->co2_mass;
  double removed_moisture = msg->moisture_content * moisture_removal_rate_;
  double removed_contaminants = msg->contaminants * contaminant_removal_rate_;

  // Publish the remaining CO2 content
  auto new_msg = demo_nova_sanctum::msg::AirData();
  new_msg.co2_mass = remaining_co2_mass;
  new_msg.moisture_content = msg->moisture_content - removed_moisture;
  new_msg.contaminants = msg->contaminants - removed_contaminants;
  
  
  new_msg.temperature = msg->temperature;
  new_msg.dew_point = msg->dew_point;

  RCLCPP_INFO(this->get_logger(),
              "Processed air data: CO2: %.2f g, Remaining Moisture: %.2f %%, Remaining Contaminants: %.2f %%",
              new_msg.co2_mass, new_msg.moisture_content, new_msg.contaminants);

  removed_moisture_publisher_->publish(new_msg);
}

void DesiccantServer::trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                       std_srvs::srv::Trigger::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "Received trigger to activate the desiccant server.");
  is_active_ = true;
  response->success = true;
  response->message = "Desiccant server activated successfully.";
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DesiccantServer>());
  rclcpp::shutdown();
  return 0;
}
