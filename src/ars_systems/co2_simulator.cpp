#include "space_station_eclss/co2_scrubber.h"
#include <algorithm>

Co2Scrubber::Co2Scrubber()
    :Node("ars_system"),
    co2_level_(declare_parameter<double>("initial_co2_level",400.0)),
    increase_rate_(declare_parameter<double>("increase_rate",5.0)),
    critical_threshold_(declare_parameter<double>("critical_threhold",800.0)),
    bake_reduction_(declare_parameter<double>("bake_reduction",300.0)),
    scrubber_efficiency_(declare_parameter<double>("scrubber_efficiency",0.9)),
    temperature_(declare_parameter<double>("station_temperature",22.0)), //degree celcius
    humidity_(declare_parameter<double>("station_humidity",50.0)) //% 



{
    ars_data_pub_=this->create_publisher<space_station_eclss::msg::ARS>("/ars_system",10);

    efficiency_service_=this->create_service<std_srvs::srv::Trigger>("/check_efficiency",
                                                                std::bind(
                &Co2Scrubber::handle_zeolite_efficiency, this, std::placeholders::_1, std::placeholders::_2));

    timer_=this->create_wall_timer(1s,std::bind(&Co2Scrubber::simulate_ars,this));


    bakery_ = this->create_client<std_srvs::srv::Trigger>("bake_gas");

    RCLCPP_INFO(this->get_logger(),"AIR REVITALIZATION SYSTEM INTIALIZED");


}


void Co2Scrubber::simulate_ars()
{
    //simulating co2, temperature, humidity 

    co2_level_ +=increase_rate_;
    temperature_ +=((rand()%100)/1000.0)-0.05; 
    humidity_ +=((rand()%100)/1000.0)-0.05; 


    if (co2_level_ >=critical_threshold_){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,  // 3 seconds
                              "Critical CO2 level reached: %.2f ppm. Immediate action required!", co2_level_);

        bake_gas();

    }

    auto msg=space_station_eclss::msg::ARS();

    msg.co2=co2_level_;
    // Temperature
    msg.temperature.temperature = temperature_;
    msg.temperature.header.stamp = this->now();  
    msg.temperature.header.frame_id = "ISS_Temp"; 
    msg.temperature.variance = 0.01;  

    // Humidity
    msg.humidity.relative_humidity = humidity_;
    msg.humidity.header.stamp = this->now(); 
    msg.humidity.header.frame_id = "ISS_Humidity"; 
    msg.humidity.variance = 0.01;  


    ars_data_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(),"Co2: %.2f ppm \n Temperature: %.2f °C \n Humidity:%.2f ",
                co2_level_,temperature_,humidity_);

}  

void Co2Scrubber::bake_gas(){

    if(!bakery_->wait_for_service(5s)){
        RCLCPP_ERROR(this->get_logger(),"Something Wrong with Scrubber");

    }

    auto request=std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result=bakery_->async_send_request(request);

    try{
        auto res=result.get();
        if(res->success){
            RCLCPP_INFO(this->get_logger(),"Baked and CO2 sent to space:%s",res->message.c_str());

        }

        else{
            RCLCPP_WARN(this->get_logger(),"Bake process Failed:%s",res->message.c_str());

        }        
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),"Exception while calling service: %s",e.what());

    }
}


void Co2Scrubber::handle_zeolite_efficiency( const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res){

    bool scrubber_success=(rand()%10) < 8 ;

    if (scrubber_success){

        double initial_co2=co2_level_;
        co2_level_*=0.7; //reducing the co2 to simulate the baking scenario


        res->success=true;
        res->message=
            "Baking CO2 successful. Initial CO2 level: " + std::to_string(initial_co2) +
            " ppm. Reduced to: " + std::to_string(co2_level_) +
            " ppm with efficiency: " + std::to_string(scrubber_efficiency_ * 100) + "%.";


        RCLCPP_INFO(this->get_logger(),"Bake request handed successfully, CO2 Reduced from %.2f ppm to %.2f ppm",
        initial_co2,co2_level_);

    }
    else{
        res->success=false;
        res->message="Baking failed due to malfunction.Please check system ASAP";


        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000,  // 3 seconds
                              "Bake request failed due to scrubber malfunction. CO2 level CRITICAL: %.2f", co2_level_);
    }
}



int main(int argc,char *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Co2Scrubber>());
    rclcpp::shutdown();
    return 0;
}