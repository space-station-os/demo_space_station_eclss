#include "space_station_eclss/co2_scrubber.h"

Co2Scrubber::Co2Scrubber()
    : Node("baking_process"),
      scrubber_efficiency_(declare_parameter<double>("scrubber_efficiency", 0.9))
{
    efficiency_service_ = this->create_service<space_station_eclss::srv::Bake>(
        "/check_efficiency",
        std::bind(&Co2Scrubber::handle_zeolite_efficiency, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "BAKING PROCESS INITIALIZED");
}

void Co2Scrubber::handle_zeolite_efficiency(
    const std::shared_ptr<space_station_eclss::srv::Bake::Request> req,
    std::shared_ptr<space_station_eclss::srv::Bake::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Bake request received. Initial CO2 level: %.2f ppm", req->co2_level);

    bool scrubber_success = (rand() % 10) < 8;  // 80% chance of success

    if (scrubber_success)
    {
        double reduced_co2 = std::max(req->co2_level * scrubber_efficiency_, 100.0);
        res->success = true;
        res->reduced_level = reduced_co2;
        res->message = "Baking CO2 successful.";
        RCLCPP_INFO(this->get_logger(),
                    "Bake request handled successfully. CO2 reduced from %.2f ppm to %.2f ppm.",
                    req->co2_level, reduced_co2);
    }
    else
    {
        res->success = false;
        res->reduced_level = req->co2_level;  // No reduction in case of failure
        res->message = "Baking failed due to malfunction.";
        RCLCPP_FATAL(this->get_logger(),
                     "Bake request failed. CO2 level remains at %.2f ppm.",
                     req->co2_level);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Co2Scrubber>());
    rclcpp::shutdown();
    return 0;
}
