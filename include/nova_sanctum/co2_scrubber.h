#ifndef CO2_SCRUBBER_HPP
#define CO2_SCRUBBER_HPP

#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "nova_sanctum/msg/ars.hpp"
#include "nova_sanctum/srv/bake.hpp"
#include <random> 

using namespace std::chrono_literals;

class Co2Scrubber : public rclcpp::Node{
    public:
        Co2Scrubber();

    private:

        void simulate_ars(); 
        void bake_gas();
        void handle_zeolite_efficiency(
            const std::shared_ptr<nova_sanctum::srv::Bake::Request>request,
            std::shared_ptr<nova_sanctum::srv::Bake::Response> response);


        //Parameters

        double co2_level_;  //Current CO2 level (ppm)
        double increase_rate_; //Rate of CO2 increase (ppm per second)
        double critical_threshold_; //CO2 threshold to trigger the bake process
        double bake_reduction_; //co2 reduction amount after baking
        double scrubber_efficiency_; //scrubber efficiency percentage 
        double temperature_; 
        double humidity_;
        bool stop_baking_;

        rclcpp::Publisher<nova_sanctum::msg::ARS>::SharedPtr ars_data_pub_;
        rclcpp::Service<nova_sanctum::srv::Bake>::SharedPtr efficiency_service_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Client<nova_sanctum::srv::Bake>::SharedPtr bakery_;


};

#endif 