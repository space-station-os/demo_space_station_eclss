#ifndef DEMO_NOVA_SANCTUM_ELECTROLYSIS_PUBLISHER_HPP_
#define DEMO_NOVA_SANCTUM_ELECTROLYSIS_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "demo_nova_sanctum/msg/process_water.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>


class Electrolysis: public rclcpp::Node
{
    public:
        Electrolysis();
        void Waterreciever(const demo_nova_sanctum::msg::ProcessWater::SharedPtr msg);
        void MicrobialRemover(const std_srvs::srv::Trigger::SharedPtr msg);
        void GasSensor();
        void PressureDome();
        void filter();
        void adiabatic_chamber();
        void coolant_chamber();

        double water_; 
        double iodine_level_;
        double gas_bubbles_; 
        double electrolysis_;
        




