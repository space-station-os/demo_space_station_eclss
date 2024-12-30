#include "nova_sanctum/waste_collector.h"

WasteCollection::WasteCollection() : Node("waste_collection") {
   
    total_volume_ = 0.0;
    source_ = 0;  

    waste_map_ = {{"Urine", 0.0}, {"Humidity Condensate", 0.0}, {"Bath/Wash", 0.0}};

    waste_collection_pub_ = this->create_publisher<nova_sanctum::msg::WasteCollection>("/waste_collection", 10);

    storage_status_pub_ = this->create_publisher<nova_sanctum::msg::StorageStatus>("/storage_status", 10);

    
    timer_ = this->create_wall_timer(5s, std::bind(&WasteCollection::simulate_waste_collection, this));

    RCLCPP_INFO(this->get_logger(), "Waste Collection Node Initialized");
}

void WasteCollection::simulate_waste_collection() {
    auto msg = nova_sanctum::msg::WasteCollection();

    // Simulate waste collection from different sources
    switch (source_) {
        case 0:
            msg.source = 0;  
            msg.volume = 50.0;  
            waste_map_["Urine"] += msg.volume;
            break;

        case 1:
            msg.source = 1;  
            msg.volume = 30.0;  
            waste_map_["Humidity Condensate"] += msg.volume;
            break;

        case 2:
            msg.source = 2;  
            msg.volume = 100.0;  
            waste_map_["Bath/Wash"] += msg.volume;
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Unknown Source");
            return;
    }

   
    total_volume_ += msg.volume;
    waste_collection_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Collected %.2f liters from source: %u. Total Volume: %.2f liters.",
                msg.volume, msg.source, total_volume_);

    
    
    auto storage_msg = nova_sanctum::msg::StorageStatus();

    
    storage_msg.tank_1 = 0.0;          
    storage_msg.tank_2 = total_volume_;  

    if (total_volume_ > 200.0) {
        storage_msg.status = 1;  
    } else {
        storage_msg.status = 0;
    }
     
   
    storage_status_pub_->publish(storage_msg);

    RCLCPP_INFO(this->get_logger(), "Tank 2 (Waste Collector) Status Updated: %.2f liters.", storage_msg.tank_2);
    source_ = (source_ == 2) ? 0 : source_ + 1;

}

// void WasteCollection::publish_storage_status() {
//     auto storage_msg = nova_sanctum::msg::StorageStatus();

    
//     storage_msg.tank_1 = 0.0;          
//     storage_msg.tank_2 = total_volume_;  

//     if (total_volume_ > 200.0) {
//         storage_msg.status = 1;  
//     } else {
//         storage_msg.status = 0;
//     }
     
   
//     storage_status_pub_->publish(storage_msg);

//     // Log the updated storage status
//     RCLCPP_INFO(this->get_logger(), "Tank 2 (Waste Collector) Status Updated: %.2f liters.", storage_msg.tank_2);
// }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WasteCollection>());
    rclcpp::shutdown();
    return 0;
}
