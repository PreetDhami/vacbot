#include "vacbot_hardware_interface/vacbot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "wiringPi.h"
#include "cmath"

namespace vacbot_hardware_interface{
    #define MAXSPEED 15.707957651346


    hardware_interface::CallbackReturn VacbotInterface::on_init(const hardware_interface::HardwareInfo & info){
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS){

            return hardware_interface::CallbackReturn::ERROR;
        }
        
        left_forward_pin = std::stoi(info_.hardware_parameters["left_forward_pin"]);
        left_backward_pin = std::stoi(info_.hardware_parameters["left_backward_pin"]);
        left_pwm_pin = std::stoi(info_.hardware_parameters["left_pwm_pin"]);
        left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        
        right_forward_pin = std::stoi(info_.hardware_parameters["right_forward_pin"]);
        right_backward_pin = std::stoi(info_.hardware_parameters["right_backward_pin"]);
        right_pwm_pin = std::stoi(info_.hardware_parameters["right_pwm_pin"]);
        right_wheel_name = info_.hardware_parameters["right_wheel_name"];

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> VacbotInterface::export_state_interfaces() {

        std::vector<hardware_interface::StateInterface> state_interfaces;

        double temp = 0.0;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            left_wheel_name, hardware_interface::HW_IF_POSITION, &temp));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            left_wheel_name, hardware_interface::HW_IF_VELOCITY, &temp));
        


        state_interfaces.emplace_back(hardware_interface::StateInterface(
            right_wheel_name, hardware_interface::HW_IF_POSITION, &temp));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            right_wheel_name, hardware_interface::HW_IF_VELOCITY, &temp));

        return state_interfaces;

    }

    std::vector<hardware_interface::CommandInterface> VacbotInterface::export_command_interfaces(){

        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            left_wheel_name, hardware_interface::HW_IF_VELOCITY, &left_cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            right_wheel_name, hardware_interface::HW_IF_VELOCITY, &right_cmd));

        return command_interfaces;
    }



    hardware_interface::CallbackReturn VacbotInterface::on_configure(const rclcpp_lifecycle::State & previous_state){
        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Configuring ...please wait...");

        if (wiringPiSetup () == -1){
            RCLCPP_FATAL(
                rclcpp::get_logger("VacbotHardware"),
                "wiringPi Setup Failed");
            return hardware_interface::CallbackReturn::ERROR;
        }

        pinMode(left_forward_pin, OUTPUT);
        pinMode(left_backward_pin, OUTPUT);
        pinMode(left_pwm_pin, PWM_OUTPUT);

        pinMode(right_forward_pin, OUTPUT);
        pinMode(right_backward_pin, OUTPUT);
        pinMode(right_pwm_pin, PWM_OUTPUT);


        digitalWrite(left_forward_pin, LOW);
        digitalWrite(left_backward_pin, LOW);
        pwmWrite(left_pwm_pin, 0);

        digitalWrite(right_forward_pin, LOW);
        digitalWrite(right_backward_pin, LOW);
        pwmWrite(right_pwm_pin, 0);


        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VacbotInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state){
        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Cleaning up ...please wait...");

        digitalWrite(left_forward_pin, LOW);
        digitalWrite(left_backward_pin, LOW);
        pwmWrite(left_pwm_pin, 0);

        digitalWrite(right_forward_pin, LOW);
        digitalWrite(right_backward_pin, LOW);
        pwmWrite(right_pwm_pin, 0);
        
        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Successfully cleaned up!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VacbotInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Activating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn VacbotInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Deactivating ...please wait...");
        RCLCPP_INFO(rclcpp::get_logger("VacbotHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;

    }

    hardware_interface::return_type VacbotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type VacbotInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
        
        if(left_cmd > 0.0){
            digitalWrite(left_forward_pin, HIGH);
            digitalWrite(left_backward_pin, LOW);
        }
        else if(left_cmd < 0.0){
            digitalWrite(left_forward_pin, LOW);
            digitalWrite(left_backward_pin, HIGH);
        }
        else{
            digitalWrite(left_forward_pin, LOW);
            digitalWrite(left_backward_pin, LOW);
        }
        int l_pwm = abs(left_cmd)/MAXSPEED;
        pwmWrite(left_pwm_pin, std::min(l_pwm, 1023));



        if(right_cmd > 0.0){
            digitalWrite(right_forward_pin, HIGH);
            digitalWrite(right_backward_pin, LOW);
        }
        else if(right_cmd < 0.0){
            digitalWrite(right_forward_pin, LOW);
            digitalWrite(right_backward_pin, HIGH);
        }
        else{
            digitalWrite(right_forward_pin, LOW);
            digitalWrite(right_backward_pin, LOW);
        }
        int r_pwm = abs(right_cmd)/MAXSPEED;
        pwmWrite(right_pwm_pin, std::min(r_pwm, 1023));


        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    vacbot_hardware_interface::VacbotInterface, hardware_interface::SystemInterface)
