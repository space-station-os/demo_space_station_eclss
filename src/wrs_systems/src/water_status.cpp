#include "space_station_eclss/water_status.h"

WaterStatus::WaterStatus() : Node("water_status"){

    purity_level_ = 0.92 ; //Initial water level (%)
    storage_= 510; //Initial tank storage
    status_ = "OK";
    water_data_pub_ = this->create_publisher<space_station_eclss::msg::ProcessWater>("/water_status", 10);
    timer_ = this->create_wall_timer(1s, std::bind(&WaterStatus::water_indicator, this));

}

void WaterStatus::water_indicator(){
    auto msg = space_station_eclss::msg::ProcessWater();
    msg.purity_level=std::round(purity_level_ * 10000.0) / 10000.0;
    msg.clean_water_available = storage_;
    msg.status = status_;
    water_data_pub_->publish(msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),5000, "Water Status: %s \n Purity Level: %.2f \n Clean Water Available: %.2f \n", msg.status.c_str(), msg.purity_level, msg.clean_water_available);
}
int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaterStatus>());
    rclcpp::shutdown();
    return 0;
}