#ifndef PROCESS_WATER_HPP
#define PROCESS_WATER_HPP


#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "nova_sanctum/msg/process_water.hpp" 
#include "nova_sanctum/msg/storage_status.hpp"
#include "nova_sanctum/srv/filteration.hpp"
#include "nova_sanctum/msg/waste_collection.hpp"

using namespace std::chrono_literals; 

class WaterProcessor: public rclcpp::Node{

    public:
        WaterProcessor();

    private:
        void get_waste_(const nova_sanctum::msg::StorageStatus::SharedPtr msg);
        void water_process(const nova_sanctum::msg::WasteCollection::SharedPtr msg);
        double processing_rate_;
        double purity_level_;
        double processed_level_;
        double waste_storage_level_;
        int status_;
        // rclcpp::Publisher<nova_sanctum::msg::ProcessWater>::SharedPtr processor_;
        
        rclcpp::Subscription<nova_sanctum::msg::WasteCollection>::SharedPtr waste_sub_;
        rclcpp::Client<nova_sanctum::srv::Filteration>::SharedPtr waste_water_process_;
        rclcpp::TimerBase::SharedPtr timer_;

};
#endif  // PRCOESS_WATER_HPP