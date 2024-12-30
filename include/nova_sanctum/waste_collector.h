#ifndef WASTE_COLLECTION_HPP
#define WASTE_COLLECTION_HPP

#include <string>
#include <unordered_map>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nova_sanctum/msg/waste_collection.hpp"
#include "nova_sanctum/msg/storage_status.hpp"

using namespace std::chrono_literals;

class WasteCollection : public rclcpp::Node {
public:
    WasteCollection();

private:
    void simulate_waste_collection();
    void publish_storage_status();

    float total_volume_;  // Total volume collected
    uint32_t source_;     // Source of the waste (0 = Urine, 1 = Humidity Condensate, 3 = Bath/Wash)

    rclcpp::Publisher<nova_sanctum::msg::WasteCollection>::SharedPtr waste_collection_pub_;
    rclcpp::Publisher<nova_sanctum::msg::StorageStatus>::SharedPtr storage_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, float> waste_map_;  // Hashmap for waste sources and volumes
};

#endif  // WASTE_COLLECTION_HPP
