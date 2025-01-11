#include "demo_nova_sanctum/adsorbent_tank.h"
#include <mutex> // For thread-safety

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
  
  co2_ = msg->co2_mass;
  moisture_content_ = msg->moisture_content;
  contaminants_ = msg->contaminants;
  temperature_ = msg->temperature;
  dew_point_ = msg->dew_point;

  // RCLCPP_INFO(this->get_logger(), "Received air data: CO2: %.2f g, Moisture: %.2f %%, Contaminants: %.2f %%",
  //             co2_, moisture_content_, contaminants_);
}

void AdsorbentBed::process_co2() {
  std::lock_guard<std::mutex> lock(data_mutex_);


  // Parameters for PD controller
  double desired_temperature = 430.0; 
  double kp = 0.5;  
  double kd = 0.1;  

  
  double error = desired_temperature - temperature_;
  double derivative = error - previous_error_;
  double adjustment = (kp * error) + (kd * derivative);

  
  temperature_ += adjustment;
  previous_error_ = error;
  
  // temperature_=temperature_+50.0;

  RCLCPP_INFO(this->get_logger(),"Adsorbent bed temperature increased to %.2f degrees Celsius.", temperature_);
  // Prepare the processed CO2 message
  if (temperature_<450.0 && temperature_>400.0){

      // Gradual CO2 reduction (10% per step)
    double co2_decrement = co2_ * co2_removal_efficiency_ * 0.1; // 10% of total CO2 removal efficiency
    double co2_to_space = co2_decrement * co2_to_space_ratio_;  // Part of CO2 sent to space
    double co2_retained = co2_decrement - co2_to_space;         // Part of CO2 retained for Sabatier

    // Update remaining CO2
    co2_ -= co2_decrement;
    if (co2_ < 0.0) {
      co2_ = 0.0; // Ensure no negative CO2 values
    }

    // Accumulate retained CO2 for Sabatier
    retained_co2_cumulative_ += co2_retained;
    demo_nova_sanctum::msg::AirData processed_msg;

    processed_msg.header.stamp = this->get_clock()->now();
    processed_msg.frame_id = "Adsorbent Bed";
    processed_msg.co2_mass = retained_co2_cumulative_; // Publish cumulative retained CO2
    processed_msg.moisture_content = moisture_content_;       
    processed_msg.contaminants = contaminants_;           
    processed_msg.temperature = temperature_;            
    processed_msg.dew_point = dew_point_;              

    RCLCPP_INFO(this->get_logger(),
                "Processed CO2: %.2f g retained (cumulative: %.2f g), %.2f g sent to space. Remaining CO2: %.2f g",
                co2_retained, retained_co2_cumulative_, co2_to_space, co2_);

    // Publish the retained CO2 (cumulative) for Sabatier
    processed_co2_publisher_->publish(processed_msg);
  }
  else if (temperature_>=450.0){
    RCLCPP_WARN(this->get_logger(),"Adsorbent bed temperature exceeded 450 degrees Celsius. Shutting down the system.");
    is_active_=false;
  }

  // Stop processing if all CO2 has been removed
  if (co2_ <= 0.0) {
    RCLCPP_INFO(this->get_logger(), "All CO2 has been processed. Total retained CO2: %.2f g.", retained_co2_cumulative_);
    is_active_=false; // Stop the timer
  }
}



void AdsorbentBed::trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                    std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (!is_active_) {
    is_active_ = true;
    response->success = true;
    response->message = "Adsorbent bed activated successfully.";
    RCLCPP_INFO(this->get_logger(), "Adsorbent bed activated.");

    timer_ = this->create_wall_timer(1s, std::bind(&AdsorbentBed::process_co2, this));
  } else {
    response->success = false;
    response->message = "Adsorbent bed is already active.";
    RCLCPP_WARN(this->get_logger(), "Adsorbent bed is already active.");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AdsorbentBed>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
