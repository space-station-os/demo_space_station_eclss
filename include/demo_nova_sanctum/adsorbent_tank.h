#ifndef DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_
#define DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_

#include <rclcpp/rclcpp.hpp>
#include <mutex> // Include for thread safety
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>

using namespace std::chrono_literals;

/**
 * @class AdsorbentBed
 * @brief ROS2 node to manage CO2 reduction in air processing.
 * 
 * The AdsorbentBed node subscribes to air data from the `/removed_moisture` topic,
 * processes the CO2 by splitting it into two streams (sent to space and retained for further processing),
 * and publishes the retained CO2 to the `/processed_co2` topic. It operates as a server
 * that can be triggered by the air collection node.
 */
class AdsorbentBed : public rclcpp::Node {
public:
  /**
   * @brief Constructor to initialize the AdsorbentBed node.
   */
  AdsorbentBed();

private:
  /**
   * @brief Callback for processing incoming air data from the `/removed_moisture` topic.
   * 
   * Ensures that moisture and contaminants are fully removed before processing CO2.
   * 
   * @param msg Shared pointer to the incoming AirData message.
   */
  void air_data_callback(const demo_nova_sanctum::msg::AirData::SharedPtr msg);

  /**
   * @brief Service callback for activating the adsorbent bed.
   * 
   * Activates the server when a trigger request is received on the `/adsorbent_server` service.
   * 
   * @param request Shared pointer to the service request.
   * @param response Shared pointer to the service response.
   */
  void trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        const std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Processes CO2 by splitting it into two streams (sent to space and retained for further use).
   */
  void process_co2();

  // Parameters
  double co2_removal_efficiency_; ///< Percentage of CO2 removed
  double co2_to_space_ratio_;     ///< Ratio of CO2 sent to space
  double retained_co2_cumulative_; ///< Cumulative retained CO2
  double previous_error_; ///< Previous error for PID controller
  // Air data variables
  double co2_;                ///< Current CO2 mass
  double moisture_content_;   ///< Current moisture content
  double contaminants_;       ///< Current contaminant level
  double temperature_;        ///< Current temperature
  double dew_point_;          ///< Current dew point

  // ROS subscribers, publishers, and servers
  rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr air_data_subscriber_; ///< Subscriber to `/removed_moisture`
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr processed_co2_publisher_; ///< Publisher to `/processed_co2`
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_; ///< Service server for `/adsorbent_server`
  rclcpp::TimerBase::SharedPtr timer_;
  // State variables
  bool is_active_; ///< Indicates whether the adsorbent bed server is active

  // Mutex for thread safety
  std::mutex data_mutex_; ///< Mutex to protect shared air data variables
};

#endif // DEMO_NOVA_SANCTUM_ADSORBENT_BED_HPP_
