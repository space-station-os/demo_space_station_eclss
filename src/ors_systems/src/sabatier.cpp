#include "demo_nova_sanctum/sabatier.h"
#include <cmath>
#include <algorithm>

Sabatier::Sabatier()
    : Node("sabatier"),
      co2_mass_(0.0),
      h2_mass_(300.0),
      moisture_content_(0.0),
      contaminants_(0.0),
      dew_point_(0.0),
      total_air_mass_(0.0),
      reactor_temp_(300.0),
      desired_temp_(300.0),
      reactor_pressure_(11.5),
      desired_pressure_(11.5),
      h2_co2_ratio_(4.0),
      ghsv_(40000.0),
      total_inlet_flow_(7.0),
      methane_yield_(0.0),
      water_yield_(0.0),
      reaction_efficiency_(0.0),
      reactor_state_(false),
      valve_state_(false),
      integral_temp_error_(0.0),
      integral_pressure_error_(0.0),
      last_temp_error_(0.0),
      last_pressure_error_(0.0),
      last_ghsv_error_(0.0) {

  ars_subscriber_ = this->create_subscription<demo_nova_sanctum::msg::AirData>(
      "/processed_co2", 10,
      std::bind(&Sabatier::process_air_data, this, std::placeholders::_1));

  sabatier_publisher_ = this->create_publisher<demo_nova_sanctum::msg::Sabatier>(
      "/sabatier", 10);

  hydrogen_subscriber_= this->create_subscription<std_msgs::msg::Float64>(
      "/hydrogen", 10,
      std::bind(&Sabatier::process_hydrogen_data, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&Sabatier::run_reactor, this));
}

void Sabatier::process_air_data(const demo_nova_sanctum::msg::AirData &msg) {
  co2_mass_ = msg.co2_mass;
  moisture_content_ = msg.moisture_content;
  contaminants_ = msg.contaminants;
  dew_point_ = msg.dew_point;
  total_air_mass_ = co2_mass_ * (1 + moisture_content_ / 100.0 + contaminants_ / 100.0);

  RCLCPP_INFO(this->get_logger(), "Received air data: CO2: %.2f g, Moisture: %.2f %%, Contaminants: %.2f %%, Dew Point: %.2f C",
              co2_mass_, moisture_content_, contaminants_, dew_point_);
}
void Sabatier::process_hydrogen_data(const std_msgs::msg::Float64 &msg) {
  h2_mass_ = msg.data;

  RCLCPP_INFO(this->get_logger(), "Received hydrogen data: H2: %.2f g", h2_mass_);
}
void Sabatier::run_reactor() {
  if (!safety_check()) {
    RCLCPP_WARN(this->get_logger(), "Safety check failed. Reactor not running.");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Running Sabatier reactor...");
  double temp_control_signal = pid_temperature(desired_temp_, reactor_temp_);
  double pressure_control_signal = pid_pressure(desired_pressure_, reactor_pressure_);
  double ghsv_control_signal = pd_ghsv(ghsv_, total_inlet_flow_);

  // Update reactor parameters based on control signals
  reactor_temp_ += temp_control_signal;
  reactor_pressure_ += pressure_control_signal;
  ghsv_ += ghsv_control_signal;

  RCLCPP_INFO(this->get_logger(), "Reactor Temp: %.2f C, Pressure: %.2f psi, GHSV: %.2f hr^-1",
              reactor_temp_, reactor_pressure_, ghsv_);

  compute_reaction();
  publish_sabatier_outputs();

  
}

void Sabatier::compute_reaction() {
  // Stoichiometric reaction: CO2 + 4H2 -> CH4 + 2H2O
  //used gpt for calculation of molar mass
  double co2_moles = co2_mass_ / 44.01;  // CO2 molar mass (g/mol)
  double h2_moles = h2_mass_ / 2.016;   // H2 molar mass (g/mol)
  double limiting_reactant_moles = std::min(co2_moles, h2_moles / 4.0);

  methane_yield_ = limiting_reactant_moles * 16.04;  // CH4 molar mass (g/mol)
  water_yield_ = limiting_reactant_moles * 2 * 18.015; // H2O molar mass (g/mol)

  reaction_efficiency_ = (limiting_reactant_moles / co2_moles) * 100.0;

  RCLCPP_INFO(this->get_logger(), "Methane Yield: %.2f g /n , Water Yield: %.2f g /n, Efficiency: %.2f %%",
              methane_yield_, water_yield_, reaction_efficiency_);
}

void Sabatier::publish_sabatier_outputs() {
  auto message = demo_nova_sanctum::msg::Sabatier();
  message.ch4_yield = methane_yield_;
  message.h2o_yield = water_yield_;
  message.temperature = reactor_temp_;
  message.pressure = reactor_pressure_;
  message.efficiency = reaction_efficiency_;

  sabatier_publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published Sabatier outputs.");
}

bool Sabatier::safety_check() {
  if (reactor_temp_ > 500.0 || reactor_temp_ < 200.0) {
    RCLCPP_WARN(this->get_logger(), "Reactor temperature out of bounds: %.2f", reactor_temp_);
    return false;
  }
  if (reactor_pressure_ > 15.0 || reactor_pressure_ < 10.0) {
    RCLCPP_WARN(this->get_logger(), "Reactor pressure out of bounds: %.2f", reactor_pressure_);
    return false;
  }
  return true;
}

// PID Controller for Temperature
double Sabatier::pid_temperature(double desired, double current) {
  double error = desired - current;
  integral_temp_error_ += error;
  double derivative = error - last_temp_error_;
  last_temp_error_ = error;

  double kp = 1.0, ki = 0.1, kd = 0.05;
  return kp * error + ki * integral_temp_error_ + kd * derivative;
}

// PID Controller for Pressure
double Sabatier::pid_pressure(double desired, double current) {
  double error = desired - current;
  integral_pressure_error_ += error;
  double derivative = error - last_pressure_error_;
  last_pressure_error_ = error;

  double kp = 1.0, ki = 0.1, kd = 0.05;
  return kp * error + ki * integral_pressure_error_ + kd * derivative;
}

// PD Controller for GHSV
double Sabatier::pd_ghsv(double desired, double current) {
  double error = desired - current;
  double derivative = error - last_ghsv_error_;
  last_ghsv_error_ = error;

  double kp = 0.5, kd = 0.1;
  return kp * error + kd * derivative;
}
int main(int argc, char *argv[]) {
  
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sabatier>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
