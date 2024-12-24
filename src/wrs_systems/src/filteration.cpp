#include "space_station_eclss/filteration.h"

Filtration::Filtration()
: Node("filtration_node"){
    // Service server for water filtration
    waste_water_process_server_ = this->create_service<space_station_eclss::srv::Filteration>(
        "/waste_water_process", std::bind(&Filtration::process_water_server_, this, std::placeholders::_1, std::placeholders::_2));

    // Publisher for water levels
    storage_status_pub_ = this->create_publisher<space_station_eclss::msg::StorageStatus>("/storage_status", 10);
    storage_sub_ = this->create_subscription<space_station_eclss::msg::StorageStatus>(
        "/storage_status", 10, std::bind(&Filtration::get_waste_, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Filtration Node Initialized");
}

void Filtration::get_waste_(const space_station_eclss::msg::StorageStatus::SharedPtr msg){
    waste_water_level_=msg->tank_2;
    clean_water_level_=msg->tank_1;
    status_=msg->status;
    RCLCPP_INFO(this->get_logger(),"Status of tank A: %f, Status of tank B: %f",clean_water_level_,waste_water_level_);

}

void Filtration::publish_water_levels() {
    auto msg = space_station_eclss::msg::StorageStatus();
    msg.tank_1 = clean_water_level_;  // Clean water tank
    msg.tank_2 = waste_water_level_;  // Waste water tank

    storage_status_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published water levels: Tank 1 (clean) = %.2f, Tank 2 (waste) = %.2f",
                clean_water_level_, waste_water_level_);
}

void Filtration::process_water_server_(
    const std::shared_ptr<space_station_eclss::srv::Filteration::Request> request,
    std::shared_ptr<space_station_eclss::srv::Filteration::Response> response) {
    float processing_rate = request->processing_rate;

    RCLCPP_INFO(this->get_logger(),"Filteration service called with processing rate: %.2f", processing_rate);

    if (waste_water_level_ <= 0) {
        response->success = false;
        response->message = "Tank 2 (waste) is empty. No processing required.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    // Simulate processing
    double processed_volume = waste_water_level_ * processing_rate;
    waste_water_level_ -= processed_volume;
    clean_water_level_ += processed_volume;

    // Ensure tanks don't exceed limits or go negative
    if (clean_water_level_ > 500) {
        clean_water_level_ = 500;
        RCLCPP_WARN(this->get_logger(), "Tank 1 (clean) is full.");
    }
    if (waste_water_level_ < 0) waste_water_level_ = 0;

    response->success = true;
    response->message = "Processing complete.";
    RCLCPP_INFO(this->get_logger(),
                "Processed %.2f liters of water. Tank 1 (clean): %.2f, Tank 2 (waste): %.2f",
                processed_volume, clean_water_level_, waste_water_level_);

    // Publish updated water levels
    publish_water_levels();
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Filtration>());
    rclcpp::shutdown();
    return 0;
}
