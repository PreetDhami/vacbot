#ifndef VACBOT_HARDWARE_INTERFACE_H
#define VACBOT_HARDWARE_INTERFACE_H


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <string>
#include <wiringPi.h>

namespace vacbot_hardware_interface{
    class VacbotInterface : public hardware_interface::SystemInterface{
        public:

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;


        private:
            uint8_t left_forward_pin;
            uint8_t left_backward_pin;
            uint8_t left_pwm_pin;
            std::string left_wheel_name = "";
            double left_cmd;

            uint8_t right_forward_pin;
            uint8_t right_backward_pin;
            uint8_t right_pwm_pin;
            std::string right_wheel_name = "";
            double right_cmd; 

    };
}


#endif