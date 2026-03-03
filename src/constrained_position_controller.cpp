
#include "aatb_controllers/constrained_position_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace aatb_controllers
{

ConstrainedPositionController::ConstrainedPositionController()
: controller_interface::ControllerInterface(),
  command_interface_name_(hardware_interface::HW_IF_POSITION),
  new_command_available_(false),
  trajectory_initialized_(false)
{
}

controller_interface::InterfaceConfiguration
ConstrainedPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + command_interface_name_);
  }

  return config;
}

controller_interface::InterfaceConfiguration
ConstrainedPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : joint_names_)
  {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }

  return config;
}

controller_interface::CallbackReturn ConstrainedPositionController::on_init()
{
  try
  {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<double>>("velocity_limits", std::vector<double>());
    auto_declare<std::vector<double>>("acceleration_limits", std::vector<double>());
    auto_declare<std::vector<double>>("jerk_limits", std::vector<double>());
    auto_declare<double>("control_cycle_time", 0.001);
    auto_declare<std::string>("interface_name", command_interface_name_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConstrainedPositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t num_joints = joint_names_.size();

  velocity_limits_ = get_node()->get_parameter("velocity_limits").as_double_array();
  acceleration_limits_ = get_node()->get_parameter("acceleration_limits").as_double_array();
  jerk_limits_ = get_node()->get_parameter("jerk_limits").as_double_array();
  control_cycle_time_ = get_node()->get_parameter("control_cycle_time").as_double();

  // Read interface name parameter
  command_interface_name_ = get_node()->get_parameter("interface_name").as_string();

  // Validate constraint dimensions
  if (velocity_limits_.size() != num_joints)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Velocity limits size (%zu) does not match number of joints (%zu)",
      velocity_limits_.size(), num_joints);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (acceleration_limits_.size() != num_joints)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Acceleration limits size (%zu) does not match number of joints (%zu)",
      acceleration_limits_.size(), num_joints);
    return controller_interface::CallbackReturn::ERROR;
  }

  if (jerk_limits_.size() != num_joints)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Jerk limits size (%zu) does not match number of joints (%zu)",
      jerk_limits_.size(), num_joints);
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize Ruckig with dynamic DOFs
  try
  {
    ruckig_ = std::make_unique<ruckig::Ruckig<ruckig::DynamicDOFs>>(num_joints, control_cycle_time_);
    ruckig_input_ = std::make_unique<ruckig::InputParameter<ruckig::DynamicDOFs>>(num_joints);
    ruckig_output_ = std::make_unique<ruckig::OutputParameter<ruckig::DynamicDOFs>>(num_joints);

    // Set motion constraints
    ruckig_input_->max_velocity = velocity_limits_;
    ruckig_input_->max_acceleration = acceleration_limits_;
    ruckig_input_->max_jerk = jerk_limits_;


    RCLCPP_INFO(
      get_node()->get_logger(),
      "Ruckig initialized with %zu DOFs, control cycle: %f s",
      num_joints, control_cycle_time_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to initialize Ruckig: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Set up command subscription
  command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { command_callback(msg); });

  // Initialize command buffer
  last_command_msg_ = std::make_shared<std_msgs::msg::Float64MultiArray>();
  rt_command_ptr_.writeFromNonRT(last_command_msg_);

  RCLCPP_INFO(get_node()->get_logger(), "Configuration complete");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConstrainedPositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear joint interface references
  joint_position_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();

  // Assign command interfaces in joint_names_ order
  for (const auto & joint_name : joint_names_)
  {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&](const hardware_interface::LoanedCommandInterface & interface) {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == command_interface_name_;
      });
    if (it == command_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Command interface '%s/%s' not found", joint_name.c_str(), command_interface_name_.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_position_command_interfaces_.emplace_back(std::ref(*it));
  }

  // Assign state interfaces in joint_names_ order
  for (const auto & joint_name : joint_names_)
  {
    auto it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const hardware_interface::LoanedStateInterface & interface) {
        return interface.get_prefix_name() == joint_name &&
               interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    if (it == state_interfaces_.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "State interface '%s/%s' not found", joint_name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return controller_interface::CallbackReturn::ERROR;
    }
    joint_position_state_interfaces_.emplace_back(std::ref(*it));
  }

  // Set command interfaces to NaN so hardware write() is a no-op
  // until the first update() cycle writes real values from Ruckig
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_position_command_interfaces_[i].get().set_value(
      std::numeric_limits<double>::quiet_NaN());
  }

  // Initialize Ruckig with current joint positions
  if (!reset_trajectory_state())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ConstrainedPositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Release interfaces
  joint_position_command_interfaces_.clear();
  joint_position_state_interfaces_.clear();

  trajectory_initialized_ = false;
  new_command_available_ = false;
  rt_command_ptr_.writeFromNonRT(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ConstrainedPositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get latest command
  auto current_command = rt_command_ptr_.readFromRT();

  // Update target if new command is available
  if (current_command && *current_command &&
      (*current_command)->data.size() == joint_names_.size())
  {

    // Update target positions
    ruckig_input_->target_position = (*current_command)->data;

    // Set zero target velocity and acceleration for position-only control
    std::fill(ruckig_input_->target_velocity.begin(),
              ruckig_input_->target_velocity.end(), 0.0);
    std::fill(ruckig_input_->target_acceleration.begin(),
              ruckig_input_->target_acceleration.end(), 0.0);

    // Clear the command to avoid reprocessing
    rt_command_ptr_.writeFromNonRT(nullptr);
  }

  // Calculate next trajectory point
  try
  {
    const auto result = ruckig_->update(*ruckig_input_, *ruckig_output_);

    // Write commanded positions to hardware
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_position_command_interfaces_[i].get().set_value(ruckig_output_->new_position[i]);
    }

    // Prepare for next cycle
    ruckig_output_->pass_to_input(*ruckig_input_);

    if (result == ruckig::Result::Finished)
    {
      RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Trajectory finished, holding position");
    }
    else if (result != ruckig::Result::Working)
    {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Ruckig returned unexpected result: %d", static_cast<int>(result));
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Ruckig update failed: %s", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void ConstrainedPositionController::command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Command dimension mismatch: received %zu, expected %zu",
      msg->data.size(), joint_names_.size());
    return;
  }

  // Check for NaN or infinite values
  for (size_t i = 0; i < msg->data.size(); ++i)
  {
    if (!std::isfinite(msg->data[i]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid command value at index %zu: %f", i, msg->data[i]);
      return;
    }
  }

  rt_command_ptr_.writeFromNonRT(msg);

  RCLCPP_DEBUG(get_node()->get_logger(), "New command received");
}

bool ConstrainedPositionController::reset_trajectory_state()
{
  if (joint_position_state_interfaces_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No state interfaces available");
    return false;
  }

  try
  {
    // Read current positions from hardware
    std::vector<double> current_positions(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      current_positions[i] = joint_position_state_interfaces_[i].get().get_value();
    }

    // Initialize Ruckig state
    ruckig_input_->current_position = current_positions;
    ruckig_input_->current_velocity = std::vector<double>(joint_names_.size(), 0.0);
    ruckig_input_->current_acceleration = std::vector<double>(joint_names_.size(), 0.0);

    // Set target to current position (hold position)
    ruckig_input_->target_position = current_positions;
    ruckig_input_->target_velocity = std::vector<double>(joint_names_.size(), 0.0);
    ruckig_input_->target_acceleration = std::vector<double>(joint_names_.size(), 0.0);

    trajectory_initialized_ = true;

    RCLCPP_INFO(get_node()->get_logger(), "Trajectory state reset to current position");
    return true;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reset trajectory: %s", e.what());
    return false;
  }
}

}  // namespace aatb_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  aatb_controllers::ConstrainedPositionController, controller_interface::ControllerInterface)