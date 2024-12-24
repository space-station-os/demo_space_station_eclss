#ifndef PROCESS_WATER_HPP
#define PROCESS_WATER_HPP


#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "space_station_eclss/msg/process_water.hpp" 
#include "space_station_eclss/msg/storage_status.hpp"
#include "space_station_eclss/srv/filteration.hpp"
#include "space_station_eclss/msg/waste_collection.hpp"

using namespace std::chrono_literals; 

class WaterProcessor: public rclcpp::Node{

    public:
        WaterProcessor();

    private:
        void get_waste_(const space_station_eclss::msg::StorageStatus::SharedPtr msg);
        void water_process(const space_station_eclss::msg::WasteCollection::SharedPtr msg);
        double processing_rate_;
        double purity_level_;
        double processed_level_;
        double waste_storage_level_;
        int status_;
        // rclcpp::Publisher<space_station_eclss::msg::ProcessWater>::SharedPtr processor_;
        
        rclcpp::Subscription<space_station_eclss::msg::WasteCollection>::SharedPtr waste_sub_;
        rclcpp::Client<space_station_eclss::srv::Filteration>::SharedPtr waste_water_process_;
        rclcpp::TimerBase::SharedPtr timer_;

};
#endif  // PRCOESS_WATER_HPP