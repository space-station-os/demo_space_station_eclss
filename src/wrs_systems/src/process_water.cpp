#include "demo_nova_sanctum/process_water.h"

WaterProcessor::WaterProcessor()
: Node ("water_processor"),
  processing_rate_(0.8)
  {
    // processor_=this->create_publisher<demo_nova_sanctum::msg::ProcessWater>("/process_water", 10);
    // storage_sub_ = this->create_subscription<demo_nova_sanctum::msg::StorageStatus>(
    //     "/storage_status", 10, std::bind(&WaterProcessor::get_waste_, this, std::placeholders::_1));

    waste_water_process_=this->create_client<demo_nova_sanctum::srv::Filteration>("/waste_water_process");

    // timer_=this->create_wall_timer(1s, std::bind(&WaterProcessor::water_process, this));
     waste_sub_ = this->create_subscription<demo_nova_sanctum::msg::WasteCollection>(
        "/waste_collection", 10, std::bind(&WaterProcessor::water_process, this, std::placeholders::_1));

    processed_level_=0.0;
    waste_storage_level_=0.0;
    status_=0;

    RCLCPP_INFO(this->get_logger(), "Water Processor Initialized");
  
  }

//   void WaterProcessor::get_waste_(const demo_nova_sanctum::msg::StorageStatus::SharedPtr msg){
//     waste_storage_level_=msg->tank_2;
//     processed_level_=msg->tank_1;
//     status_=msg->status;
//     RCLCPP_INFO(this->get_logger(),"Status of tank A: %f, Status of tank B: %f",processed_level_,waste_storage_level_);

//   }

  void WaterProcessor::water_process(const demo_nova_sanctum::msg::WasteCollection::SharedPtr msg) {
    
    RCLCPP_INFO(this->get_logger(), "Processing waste water from source: %u", msg->source);
    switch (msg->source) {
        case 0: // Urine Processing
            processing_rate_ = 0.5; // 50% efficiency
            break;

        case 1: // Sweat Processing
            processing_rate_ = 1.0; // 100% efficiency
            break;

        case 2: // Hygiene Water Processing
            processing_rate_ = 2.0; // 200% efficiency
            break;

        case 3: // Clothes Wash Water Processing
            processing_rate_ = 1.5; // 150% efficiency
            break;

        case 4: // Dish Wash Water Processing
            processing_rate_ = 1.0; // 100% efficiency
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Unknown waste type. No processing performed.");
            return;
    }

    
    if (!waste_water_process_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available.");
        return;
    }

    
    auto request = std::make_shared<demo_nova_sanctum::srv::Filteration::Request>();
    request->processing_rate = processing_rate_;

    
    auto future = waste_water_process_->async_send_request(request);

    
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Processing successful: %s", response->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Processing failed: %s", response->message.c_str());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
}


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaterProcessor>());
  rclcpp::shutdown();
  return 0;
}