#ifndef DEMO_NOVA_SANCTUM_DESICCANT_TANK_HPP_
#define DEMO_NOVA_SANCTUM_DESICCANT_TANK_HPP_

#include <rclcpp/rclcpp.hpp>
#include "demo_nova_sanctum/msg/air_data.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>

using namespace std::chrono_literals; // Enables usage of chrono literals like 1s, 500ms, etc.

/**
 * @class DesiccantServer
 * @brief ROS2 node for processing air data by removing moisture and contaminants.
 * 
 * The DesiccantServer node subscribes to air data from the `/unpure_air` topic,
 * processes the data by removing moisture and contaminants, and publishes the
 * processed data to the `/removed_moisture` topic. It can be activated via a 
 * service call to `/desiccant_server`.
 */
class DesiccantServer : public rclcpp::Node {
public:
  /**
   * @brief Constructor to initialize the DesiccantServer node.
   */
  DesiccantServer();

private:
  /**
   * @brief Callback function for processing incoming air data messages.
   * 
   * This function is triggered whenever a message is received on the `/unpure_air` topic.
   * It stores the data for processing and triggers further actions if the server is active.
   * 
   * @param msg Shared pointer to the incoming AirData message.
   */
  void air_data_callback(const demo_nova_sanctum::msg::AirData::SharedPtr msg);

  /**
   * @brief Processes the stored air data by removing moisture and contaminants.
   * 
   * This function calculates the processed air data and publishes it to the `/removed_moisture` topic.
   */
  void process_air_data();

  /**
   * @brief Service callback for activating the desiccant server.
   * 
   * This function activates the server when a trigger request is received on the `/desiccant_server` service.
   * 
   * @param request Shared pointer to the service request.
   * @param response Shared pointer to the service response.
   */
  void trigger_callback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                        const std_srvs::srv::Trigger::Response::SharedPtr response);

  void trigger_server(const std::string &server_name, const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client);
  
  void deactivate();

  // Parameters for removal efficiency
  double moisture_removal_rate_;    ///< Percentage of moisture removed (e.g., 95% = 0.95)
  double contaminant_removal_rate_; ///< Percentage of contaminants removed (e.g., 90% = 0.90)

  // Air data variables
  double co2_;                ///< Current CO2 mass
  double moisture_content_;   ///< Current moisture content
  double contaminants_;       ///< Current contaminant level
  double temperature_;        ///< Current temperature
  double dew_point_;          ///< Current dew point

  // ROS Interfaces
  rclcpp::Subscription<demo_nova_sanctum::msg::AirData>::SharedPtr air_data_subscriber_; ///< Subscriber to `/unpure_air`
  rclcpp::Publisher<demo_nova_sanctum::msg::AirData>::SharedPtr removed_moisture_publisher_; ///< Publisher to `/removed_moisture`
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_server_; ///< Service server for `/desiccant_server`
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic operations
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr adsorbent_server_client_;

  // State variables
  bool is_active_; ///< Indicates whether the desiccant server is active

  std::mutex data_mutex_; ///< Mutex to protect shared air data variables
};

#endif // DEMO_NOVA_SANCTUM_DESICCANT_TANK_HPP_



//USED CHATGPT TO ADD COMMENTS FOR BETTER EXPLANATION