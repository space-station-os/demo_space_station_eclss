#ifndef WATER_STATUS_HPP
#define WATER_STATUS_HPP
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "space_station_eclss/msg/process_water.hpp"
#include <string>


using namespace std::chrono_literals;


class WaterStatus : public rclcpp::Node{
    public:
        WaterStatus();

    private:
        void water_indicator();

        //Parameters
        double purity_level_; //Current water level (gallon)
        double storage_; //Current status of the water system (Normal, Warning, Critical)
        std::string status_;
        rclcpp::Publisher<space_station_eclss::msg::ProcessWater>::SharedPtr water_data_pub_;
        rclcpp::TimerBase::SharedPtr timer_; 
};

#endif // WATER_STATUS_HPP