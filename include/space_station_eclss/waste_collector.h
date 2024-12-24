#ifndef WASTE_COLLECTION_HPP
#define WASTE_COLLECTION_HPP

#include <string>
#include <unordered_map>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "space_station_eclss/msg/waste_collection.hpp"
#include "space_station_eclss/msg/storage_status.hpp"

using namespace std::chrono_literals;

class WasteCollection : public rclcpp::Node {
public:
    WasteCollection();

private:
    void simulate_waste_collection();
    void publish_storage_status();

    float total_volume_;  // Total volume collected
    uint32_t source_;     // Source of the waste (0 = Urine, 1 = Humidity Condensate, 3 = Bath/Wash)

    rclcpp::Publisher<space_station_eclss::msg::WasteCollection>::SharedPtr waste_collection_pub_;
    rclcpp::Publisher<space_station_eclss::msg::StorageStatus>::SharedPtr storage_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, float> waste_map_;  // Hashmap for waste sources and volumes
};

#endif  // WASTE_COLLECTION_HPP
