#include "so100_arm/so100_arm_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace so100_arm
{
SO100ArmInterface::SO100BidirectionalInterface() 
{
}

SO100ArmInterface::~SO100BidirectionalInterface()
{
    if (use_serial_) {
        st3215_.end();
    }
}

CallbackReturn SO100ArmInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    // Debug: Print all received parameters
    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), "Hardware parameters received:");
    for (const auto& param : hardware_info.hardware_parameters) {
        RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                   "  %s = %s", param.first.c_str(), param.second.c_str());
    }

    use_serial_ = hardware_info.hardware_parameters.count("use_serial") ?
        (hardware_info.hardware_parameters.at("use_serial") == "true" || 
         hardware_info.hardware_parameters.at("use_serial") == "True") : false;
    
    serial_port_ = hardware_info.hardware_parameters.count("serial_port") ?
        hardware_info.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";
    
    serial_baudrate_ = hardware_info.hardware_parameters.count("serial_baudrate") ?
        std::stoi(hardware_info.hardware_parameters.at("serial_baudrate")) : 1000000;

    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
               "Configuration: use_serial=%s, port=%s, baudrate=%d", 
               use_serial_ ? "true" : "false", serial_port_.c_str(), serial_baudrate_);

    size_t num_joints = info_.joints.size();
    position_commands_.resize(num_joints, 0.0);
    position_states_.resize(num_joints, 0.0);
    velocity_states_.resize(num_joints, 0.0);
    
    // Initialize positions to reasonable home values for fake hardware mode
    if (!use_serial_) {
        // Set to reasonable home positions (all joints at 0 radians)
        std::fill(position_states_.begin(), position_states_.end(), 0.0);
        std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SO100ArmInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SO100ArmInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }
    return command_interfaces;
}

CallbackReturn SO100ArmInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), "Activating so100_bidirectional hardware interface...");

    if (use_serial_) {
        RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                   "Attempting to connect to servos on %s at %d baud", serial_port_.c_str(), serial_baudrate_);
        
        if(!st3215_.begin(serial_baudrate_, serial_port_.c_str())) {
            RCLCPP_ERROR(rclcpp::get_logger("SO100ArmInterface"), 
                        "Failed to initialize serial connection to %s", serial_port_.c_str());
            return CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                   "Serial connection established, testing servo communication...");

        // Initialize each servo
        bool any_servo_failed = false;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                       "Pinging servo %d...", servo_id);
            
            // First ping the servo with multiple attempts
            bool ping_success = false;
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (st3215_.Ping(servo_id) != -1) {
                    ping_success = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            if (!ping_success) {
                RCLCPP_ERROR(rclcpp::get_logger("SO100ArmInterface"), 
                            "No response from servo %d after 3 attempts - check robot connection and power", servo_id);
                any_servo_failed = true;
                continue;  // Try other servos
            }
            
            RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                       "Servo %d responded successfully", servo_id);
            
            // First disable torque before changing mode
            if (!st3215_.EnableTorque(servo_id, 0)) {
                RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                           "Failed to disable torque for servo %d before mode change", servo_id);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            // Set to position control mode
            if (!st3215_.Mode(servo_id, 0)) {
                RCLCPP_ERROR(rclcpp::get_logger("SO100ArmInterface"), 
                            "Failed to set mode for servo %d", servo_id);
                // Try to continue with other servos instead of failing completely
                RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                           "Continuing initialization with remaining servos...");
                continue;
            }
            
            // Re-enable torque after mode change
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (!st3215_.EnableTorque(servo_id, 1)) {
                RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                           "Failed to re-enable torque for servo %d after mode change", servo_id);
            }

            // Read initial position and set command to match (no movement)
            if (st3215_.FeedBack(servo_id) != -1) {
                int pos = st3215_.ReadPos(servo_id);
                position_states_[i] = ticks_to_radians(pos, i);
                position_commands_[i] = position_states_[i];  // Match current position exactly
                
                // DO NOT send any position commands during initialization to prevent movement
                // The servo will naturally hold its current position in position control mode
                
                RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                           "Servo %d initialized at position %d ticks (%.3f rad) - no commands sent", 
                           servo_id, pos, position_states_[i]);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Check if we should fall back to simulation mode
        if (any_servo_failed) {
            RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                       "Some servos failed to initialize - falling back to simulation mode");
            RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                       "Check robot power, connections, and servo IDs");
            use_serial_ = false;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                        "Serial communication initialized on %s", serial_port_.c_str());
        }
    }

    node_ = rclcpp::Node::make_shared("so100_bidirectional_driver");

    // Add services for calibration
    calib_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "record_position",
        std::bind(&SO100ArmInterface::calibration_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));
                  
    torque_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "toggle_torque",
        std::bind(&SO100ArmInterface::torque_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    // Load calibration if available
    std::string calib_file = info_.hardware_parameters.count("calibration_file") ?
        info_.hardware_parameters.at("calibration_file") : "";
        
    if (!calib_file.empty()) {
        if (!load_calibration(calib_file)) {
            RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                       "Failed to load calibration file: %s", calib_file.c_str());
        }
    }

    // Move to home position if calibration is valid
    if (calibration_valid_) {
        move_to_home_position();
        
        // Force a read cycle to update position_states_ with actual servo positions
        RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                   "Updating position states after home positioning...");
        
        // Read actual positions from servos to ensure position_states_ is accurate
        if (use_serial_) {
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                uint8_t servo_id = static_cast<uint8_t>(i + 1);
                
                // Read actual position from servo
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if (st3215_.FeedBack(servo_id) != -1) {
                    int actual_pos = st3215_.ReadPos(servo_id);
                    position_states_[i] = ticks_to_radians(actual_pos, i);
                    
                    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                               "Joint %s: actual position %d ticks (%.3f rad)", 
                               info_.joints[i].name.c_str(), actual_pos, position_states_[i]);
                }
            }
        }
        
        // Give extra time for joint state broadcaster to pick up the new positions
        RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                   "Waiting for joint state broadcaster to update...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
    } else {
        RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                   "No valid calibration loaded - skipping home positioning");
    }

    // Mark initialization as complete - now safe to accept movement commands
    initialization_complete_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), "Hardware interface activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SO100ArmInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
    // Reset initialization flag to prevent movement during shutdown
    initialization_complete_ = false;
    
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            st3215_.EnableTorque(servo_id, 0);
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), "Hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SO100ArmInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // In fake hardware mode, accept commands even without calibration for simulation
    if (use_serial_) {
        // Prevent movement until robot is properly calibrated and initialization is complete
        if (!calibration_valid_ || !initialization_complete_) {
            return hardware_interface::return_type::OK;  // Silently ignore commands
        }
        
        if (torque_enabled_) {
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                uint8_t servo_id = static_cast<uint8_t>(i + 1);
                int joint_pos_cmd = radians_to_ticks(position_commands_[i], i);
                
                if (!st3215_.RegWritePosEx(servo_id, joint_pos_cmd, 4500, 255)) {
                    // Only warn occasionally to avoid spam
                    static int write_warn_counter = 0;
                    write_warn_counter++;
                    if (write_warn_counter >= 50) {  // Warn every ~0.5 seconds
                        RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                                   "Failed to write position to servo %d", servo_id);
                        write_warn_counter = 0;
                    }
                }
            }
            st3215_.RegWriteAction();
        }
    }
    // For fake hardware mode, commands are handled in the read() method

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SO100ArmInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (use_serial_) {
        // Reduce communication frequency to avoid overruns - only read every 5th cycle (50ms)
        static int read_counter = 0;
        read_counter++;
        
        if (read_counter >= 5) {
            read_counter = 0;
            
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                uint8_t servo_id = static_cast<uint8_t>(i + 1);
                
                // Minimal delay between servo reads (reduced from 10ms to 2ms)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));

                if (!torque_enabled_) {
                    // When torque is disabled, only try to read position
                    int raw_pos = st3215_.ReadPos(servo_id);
                    if (raw_pos != -1) {
                        position_states_[i] = ticks_to_radians(raw_pos, i);
                    }
                    continue;
                }

                // Full feedback read when torque is enabled
                if (st3215_.FeedBack(servo_id) != -1) {
                    int raw_pos = st3215_.ReadPos(servo_id);
                    position_states_[i] = ticks_to_radians(raw_pos, i);
                } else {
                    // Only warn occasionally to avoid spam
                    static int warn_counter = 0;
                    warn_counter++;
                    if (warn_counter >= 50) {  // Warn every ~2.5 seconds
                        RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                                   "Failed to read feedback from servo %d", servo_id);
                        warn_counter = 0;
                    }
                }
            }
        }
    } else {
        // In fake hardware mode, maintain current positions
        // This ensures joint states are still published for visualization
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            // Keep position_states_ as is, or gradually move towards position_commands_
            if (calibration_valid_) {
                // Smoothly move towards commanded position for fake hardware simulation
                double error = position_commands_[i] - position_states_[i];
                position_states_[i] += error * 0.1;  // 10% of error each cycle
            }
        }
    }

    return hardware_interface::return_type::OK;
}

double SO100ArmInterface::ticks_to_radians(int ticks, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];
        // Convert to normalized position first (0 to 1)
        double normalized = (double)(ticks - calib.min_ticks) / calib.range_ticks;
        // Then convert to radians (-π to π)
        return (normalized * 2.0 - 1.0) * M_PI;
    }
    
    // Fallback: assume 4096 ticks per full rotation, centered at 2048
    return (ticks - 2048) * 2 * M_PI / 4096.0;
}

int SO100ArmInterface::radians_to_ticks(double radians, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];
        // Convert from radians (-π to π) to normalized position (0 to 1)
        double normalized = (radians / M_PI + 1.0) / 2.0;
        // Then convert to ticks
        return calib.min_ticks + (int)(normalized * calib.range_ticks);
    }
    
    // Fallback: assume 4096 ticks per full rotation, centered at 2048
    return 2048 + (int)(radians * 4096.0 / (2 * M_PI));
}

void SO100ArmInterface::record_current_position() 
{
    std::stringstream ss;
    ss << "{";
    
    bool first = true;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        uint8_t servo_id = static_cast<uint8_t>(i + 1);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Try multiple times to read the servo
        int pos = -1;
        for (int retry = 0; retry < 3 && pos == -1; retry++) {
            st3215_.FeedBack(servo_id);
            pos = st3215_.ReadPos(servo_id);
            if (pos == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        if (!first) {
            ss << ",";
        }
        first = false;
        
        ss << "\"" << info_.joints[i].name << "\": {"
           << "\"ticks\": " << (pos != -1 ? pos : 0) << ","
           << "\"speed\": " << st3215_.ReadSpeed(servo_id) << ","
           << "\"load\": " << st3215_.ReadLoad(servo_id)
           << "}";
    }
    ss << "}";
    
    last_calibration_data_ = ss.str();
    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                "Recorded positions: %s", last_calibration_data_.c_str());
}

void SO100ArmInterface::calibration_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    record_current_position();
    response->success = true;
    response->message = last_calibration_data_;
    
    // After calibration is completed and saved externally, user needs to restart 
    // the hardware interface to reload the new calibration file
}

void SO100ArmInterface::set_torque_enable(bool enable) 
{
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            if (!enable) {
                // Disable torque: set to idle mode, then disable
                st3215_.Mode(servo_id, 2);  // Mode 2 = idle
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                st3215_.EnableTorque(servo_id, 0);
            } else {
                // Enable torque: set to position mode, then enable
                st3215_.Mode(servo_id, 0);  // Mode 0 = position
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                st3215_.EnableTorque(servo_id, 1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        torque_enabled_ = enable;
        
        RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                    "Torque %s for all servos", enable ? "enabled" : "disabled");
    }
}

void SO100ArmInterface::move_to_home_position()
{
    if (!use_serial_ || !calibration_valid_) {
        RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                   "Cannot move to home position: serial not available or calibration invalid");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), "Moving robot to home position...");

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& joint_name = info_.joints[i].name;
        uint8_t servo_id = static_cast<uint8_t>(i + 1);
        
        if (joint_calibration_.count(joint_name) > 0) {
            const auto& calib = joint_calibration_[joint_name];
            int home_pos = calib.home_ticks;
            
            // Move to home position with moderate speed
            if (!st3215_.RegWritePosEx(servo_id, home_pos, 2000, 255)) {
                RCLCPP_ERROR(rclcpp::get_logger("SO100ArmInterface"), 
                            "Failed to set home position for servo %d (%s)", servo_id, joint_name.c_str());
            } else {
                // Update command and state to match home position
                position_states_[i] = ticks_to_radians(home_pos, i);
                position_commands_[i] = position_states_[i];
                
                RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                           "Set home position for %s: %d ticks (%.3f rad)", 
                           joint_name.c_str(), home_pos, position_states_[i]);
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                       "No calibration data found for joint %s, skipping home positioning", joint_name.c_str());
        }
    }
    
    // Execute all movements
    st3215_.RegWriteAction();
    
    // Wait for movement to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    
    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), "Robot moved to home position");
}

void SO100ArmInterface::torque_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    bool new_state = !torque_enabled_;
    
    response->success = true;
    response->message = std::string("Torque ") + (new_state ? "enabled" : "disabled");
    
    set_torque_enable(new_state);
    
    RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                "Torque service called, response: %s", response->message.c_str());
}

bool SO100ArmInterface::load_calibration(const std::string& filepath) 
{
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        auto joints = config["joints"];
        if (!joints) {
            RCLCPP_ERROR(rclcpp::get_logger("SO100ArmInterface"), 
                        "No joints section in calibration file");
            return false;
        }

        for (const auto& joint : joints) {
            std::string name = joint.first.as<std::string>();
            const auto& data = joint.second;
            
            if (!data["min"] || !data["center"] || !data["max"]) {
                RCLCPP_ERROR(rclcpp::get_logger("SO100ArmInterface"), 
                            "Missing calibration data for joint %s", name.c_str());
                continue;
            }

            JointCalibration calib;
            calib.min_ticks = data["min"]["ticks"].as<int>();
            calib.center_ticks = data["center"]["ticks"].as<int>();
            calib.max_ticks = data["max"]["ticks"].as<int>();
            calib.home_ticks = data["home"] ? data["home"]["ticks"].as<int>() : calib.center_ticks;
            calib.range_ticks = calib.max_ticks - calib.min_ticks;
            
            joint_calibration_[name] = calib;
            
            RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                       "Loaded calibration for %s: min=%d, center=%d, max=%d, home=%d", 
                       name.c_str(), calib.min_ticks, calib.center_ticks, calib.max_ticks, calib.home_ticks);
            
            // Check if this is valid calibration data (not all zeros)
            if (calib.min_ticks != 0 || calib.center_ticks != 0 || calib.max_ticks != 0) {
                calibration_valid_ = true;
            }
        }
        
        if (calibration_valid_) {
            RCLCPP_INFO(rclcpp::get_logger("SO100ArmInterface"), 
                       "Valid calibration loaded - robot movement enabled");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                       "No valid calibration found - robot movement disabled for safety");
            RCLCPP_WARN(rclcpp::get_logger("SO100ArmInterface"), 
                       "Please run calibration script to enable robot movement");
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SO100ArmInterface"), 
                    "Failed to load calibration: %s", e.what());
        return false;
    }
}

}  // namespace so100_arm

PLUGINLIB_EXPORT_CLASS(so100_arm::SO100ArmInterface, hardware_interface::SystemInterface)
