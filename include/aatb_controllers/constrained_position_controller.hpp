
#ifndef AATB_CONTROLLERS__CONSTRAINED_POSITION_CONTROLLER_HPP_
#define AATB_CONTROLLERS__CONSTRAINED_POSITION_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "aatb_controllers/visibility_control.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ruckig/ruckig.hpp"

namespace aatb_controllers
{

class ConstrainedPositionController : public controller_interface::ControllerInterface
{
public:
  AATB_CONTROLLERS_PUBLIC
  ConstrainedPositionController();

  AATB_CONTROLLERS_PUBLIC
  ~ConstrainedPositionController() = default;

  AATB_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  AATB_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  AATB_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  AATB_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  AATB_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  AATB_CONTROLLERS_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  AATB_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<std::string> joint_names_;
  std::string command_interface_name_;

  // Motion constraints
  std::vector<double> velocity_limits_;
  std::vector<double> acceleration_limits_;
  std::vector<double> jerk_limits_;
  double control_cycle_time_;

  // Ruckig trajectory generator
  std::unique_ptr<ruckig::Ruckig<ruckig::DynamicDOFs>> ruckig_;
  std::unique_ptr<ruckig::InputParameter<ruckig::DynamicDOFs>> ruckig_input_;
  std::unique_ptr<ruckig::OutputParameter<ruckig::DynamicDOFs>> ruckig_output_;

  // Command subscription
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;

  // Realtime buffer for commands
  realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>> rt_command_ptr_;
  std::shared_ptr<std_msgs::msg::Float64MultiArray> last_command_msg_;

  // Speed scaling (optional — hardware state interface or topic)
  std::string speed_scaling_state_interface_name_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    speed_scaling_state_interface_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_scaling_subscriber_;
  realtime_tools::RealtimeBuffer<double> rt_speed_scaling_;

  // State tracking
  bool new_command_available_;
  bool trajectory_initialized_;

  // Joint handles references
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interfaces_;

private:
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  bool reset_trajectory_state(bool preserve_target = false);
};

}  // namespace aatb_controllers

#endif  // AATB_CONTROLLERS__CONSTRAINED_POSITION_CONTROLLER_HPP_