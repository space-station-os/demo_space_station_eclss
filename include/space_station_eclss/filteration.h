#ifndef FILTRATION_HPP
#define FILTRATION_HPP
#include <chrono>
#include <memory>
#include "space_station_eclss/srv/filteration.hpp"
#include "space_station_eclss/msg/storage_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Filtration : public rclcpp::Node{
public:
    Filtration();

private:
    void process_water_server_(
        const std::shared_ptr<space_station_eclss::srv::Filteration::Request> request,
        std::shared_ptr<space_station_eclss::srv::Filteration::Response> response);

    void publish_water_levels();

    rclcpp::Publisher<space_station_eclss::msg::StorageStatus>::SharedPtr storage_status_pub_;

    double waste_water_level_; // Simulate the waste water level
    double clean_water_level_; // Simulate the clean water level

    rclcpp::Service<space_station_eclss::srv::Filteration>::SharedPtr waste_water_process_server_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // FILTRATION_HPP
