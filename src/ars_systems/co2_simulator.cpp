#include "demo_nova_sanctum/co2_scrubber.h"

Co2Scrubber::Co2Scrubber()
    : Node("ars_system"),
      co2_level_(declare_parameter<double>("initial_co2_level", 400.0)),
      increase_rate_(declare_parameter<double>("increase_rate", 5.0)),
      critical_threshold_(declare_parameter<double>("critical_threshold", 650.0)),
      bake_reduction_(declare_parameter<double>("bake_reduction", 300.0)),
      scrubber_efficiency_(declare_parameter<double>("scrubber_efficiency", 0.9)),
      temperature_(declare_parameter<double>("station_temperature", 22.0)),
      humidity_(declare_parameter<double>("station_humidity", 50.0))
{
    ars_data_pub_ = this->create_publisher<demo_nova_sanctum::msg::ARS>("/ars_system", 10);

    timer_ = this->create_wall_timer(
        1s, std::bind(&Co2Scrubber::simulate_ars, this));

    bakery_ = this->create_client<demo_nova_sanctum::srv::Bake>("/check_efficiency");

    RCLCPP_INFO(this->get_logger(), "AIR REVITALIZATION SYSTEM INITIALIZED");
}

void Co2Scrubber::simulate_ars()
{
    // Simulate CO2, temperature, and humidity
    co2_level_ += increase_rate_;
    temperature_ += ((rand() % 100) / 1000.0) - 0.05; 
    humidity_ += ((rand() % 100) / 1000.0) - 0.05;

    if (co2_level_ >= critical_threshold_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                              "Critical CO2 level reached: %.2f ppm. Immediate action required!", co2_level_);
        bake_gas();
    }

    // Publish ARS data
    auto msg = demo_nova_sanctum::msg::ARS();
    msg.co2 = co2_level_;
    msg.temperature.temperature = temperature_;
    msg.temperature.header.stamp = this->now();
    msg.temperature.variance = 0.01;
    msg.humidity.relative_humidity = humidity_;
    msg.humidity.header.stamp = this->now();
    msg.humidity.variance = 0.01;

    ars_data_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Co2: %.2f ppm \n Temperature: %.2f °C \n Humidity: %.2f",
                co2_level_, temperature_, humidity_);
}

void Co2Scrubber::bake_gas()
{
    if (!bakery_->wait_for_service(5s))
    {
        RCLCPP_ERROR(this->get_logger(), "Bake service not available. Make sure the server is running.");
        return;
    }

    auto request = std::make_shared<demo_nova_sanctum::srv::Bake::Request>();
    request->co2_level = co2_level_;

    auto result_future = bakery_->async_send_request(
        request,
        [this](rclcpp::Client<demo_nova_sanctum::srv::Bake>::SharedFuture future_response) {
            try
            {
                auto res = future_response.get();
                if (res->success)
                {
                    double previous_co2 = co2_level_;
                    co2_level_ = res->reduced_level; 
                    critical_threshold_ = co2_level_;
                    if (critical_threshold_ <= 100.0)
                    {
                        critical_threshold_ = 650.0;
                    }
                    RCLCPP_INFO(this->get_logger(),
                                "Bake process successful: %s Initial CO2 level: %.2f ppm. Reduced to: %.2f ppm.",
                                res->message.c_str(), previous_co2, co2_level_);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Bake process failed: %s", res->message.c_str());
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception while calling bake service: %s", e.what());
            }
        });
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Co2Scrubber>());
    rclcpp::shutdown();
    return 0;
}
