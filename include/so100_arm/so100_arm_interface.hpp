#ifndef SO100_ARM_INTERFACE_H
#define SO100_ARM_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>

#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <vector>
#include <string>
#include <memory>
#include <termios.h>
#include <map>

#include <sensor_msgs/msg/joint_state.hpp>
#include <SCServo_Linux/SCServo.h>
#include "std_srvs/srv/trigger.hpp"
#include <yaml-cpp/yaml.h>

namespace so100_arm
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SO100ArmInterface : public hardware_interface::SystemInterface
{
public:
  SO100ArmInterface();
  virtual ~SO100ArmInterface();

  // LifecycleNodeInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // SystemInterface
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Position command and state storage for all joints
  std::vector<double> position_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // Calibration data
  struct JointCalibration {
    int min_ticks;
    int center_ticks;
    int max_ticks;
    int home_ticks;
    double range_ticks;
  };
  std::map<std::string, JointCalibration> joint_calibration_;

  // Communication configuration
  bool use_serial_;
  std::string serial_port_;
  int serial_baudrate_;

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;

  SMS_STS st3215_;

  // Calibration methods
  double ticks_to_radians(int ticks, size_t servo_idx);
  int radians_to_ticks(double radians, size_t servo_idx);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr torque_service_;

  void calibration_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  void torque_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void record_current_position();
  void set_torque_enable(bool enable);
  void move_to_home_position();

  std::string last_calibration_data_;
  bool torque_enabled_{true};
  bool calibration_valid_{false};  // Prevent movement until calibrated
  bool initialization_complete_{false};  // Prevent movement during startup

  bool load_calibration(const std::string& filepath);
};

}  // namespace so100_arm

#endif  // SO100_BIDIRECTIONAL_INTERFACE_H
