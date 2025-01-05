#ifndef PROCESS_WATER_HPP
#define PROCESS_WATER_HPP


#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "demo_nova_sanctum/msg/process_water.hpp" 
#include "demo_nova_sanctum/msg/storage_status.hpp"
#include "demo_nova_sanctum/srv/filteration.hpp"
#include "demo_nova_sanctum/msg/waste_collection.hpp"

using namespace std::chrono_literals; 

class WaterProcessor: public rclcpp::Node{

    public:
        WaterProcessor();

    private:
        void get_waste_(const demo_nova_sanctum::msg::StorageStatus::SharedPtr msg);
        void water_process(const demo_nova_sanctum::msg::WasteCollection::SharedPtr msg);
        double processing_rate_;
        double purity_level_;
        double processed_level_;
        double waste_storage_level_;
        int status_;
        // rclcpp::Publisher<demo_nova_sanctum::msg::ProcessWater>::SharedPtr processor_;
        
        rclcpp::Subscription<demo_nova_sanctum::msg::WasteCollection>::SharedPtr waste_sub_;
        rclcpp::Client<demo_nova_sanctum::srv::Filteration>::SharedPtr waste_water_process_;
        rclcpp::TimerBase::SharedPtr timer_;

};
#endif  // PRCOESS_WATER_HPP