#ifndef CO2_SCRUBBER_HPP
#define CO2_SCRUBBER_HPP

#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "space_station_eclss/msg/ars.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <random> 

using namespace std::chrono_literals;

class Co2Scrubber : public rclcpp::Node{
    public:
        Co2Scrubber();

    private:

        void simulate_ars(); 
        void bake_gas();
        void handle_zeolite_efficiency(
            const std::shared_ptr<std_srvs::srv::Trigger::Request>request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);


        //Parameters

        double co2_level_;  //Current CO2 level (ppm)
        double increase_rate_; //Rate of CO2 increase (ppm per second)
        double critical_threshold_; //CO2 threshold to trigger the bake process
        double bake_reduction_; //co2 reduction amount after baking
        double scrubber_efficiency_; //scrubber efficiency percentage 
        double temperature_; 
        double humidity_;

        rclcpp::Publisher<space_station_eclss::msg::ARS>::SharedPtr ars_data_pub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr efficiency_service_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bakery_;


};

#endif 