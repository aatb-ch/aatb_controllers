
#include "aatb_controllers/constrained_position_controller.hpp"

#include <algorithm>
#include <cmath>
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

  if (!speed_scaling_state_interface_name_.empty())
  {
    config.names.push_back(speed_scaling_state_interface_name_);
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
    auto_declare<std::string>("speed_scaling.state_interface", "");
    auto_declare<double>("max_tracking_error", 0.35);
    auto_declare<std::vector<double>>("position_limits.min", std::vector<double>());
    auto_declare<std::vector<double>>("position_limits.max", std::vector<double>());
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

  // Speed scaling
  speed_scaling_state_interface_name_ =
    get_node()->get_parameter("speed_scaling.state_interface").as_string();
  rt_speed_scaling_.writeFromNonRT(1.0);

  max_tracking_error_ = get_node()->get_parameter("max_tracking_error").as_double();

  // Hard joint window, absolute radians, enforced in the control loop.
  //
  // Ruckig's own max_position/min_position are NOT usable here: in this build
  // they are only read by calculator_online.hpp (the Pro online calculator), so
  // setting them would look like a safety bound and do nothing. We enforce the
  // window ourselves instead, on the target AND on the value actually written,
  // so no path reaches the hardware without passing it.
  position_min_ = get_node()->get_parameter("position_limits.min").as_double_array();
  position_max_ = get_node()->get_parameter("position_limits.max").as_double_array();
  if (position_min_.size() != position_max_.size() ||
      (!position_min_.empty() && position_min_.size() != num_joints))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "position_limits.min/max must be empty or have %zu entries (got %zu/%zu)",
      num_joints, position_min_.size(), position_max_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < position_min_.size(); ++i)
  {
    if (!(position_min_[i] < position_max_[i]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "position_limits for '%s' are not ordered: min %.4f >= max %.4f",
        joint_names_[i].c_str(), position_min_[i], position_max_[i]);
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (position_min_.empty())
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "No position_limits configured -- the controller will not bound the "
      "commanded position. Set position_limits.min/max to the joint windows.");
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Position window enforced on %zu joints; max_tracking_error %.3f rad",
      position_min_.size(), max_tracking_error_);
  }

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

  // Speed scaling topic subscription (fallback when no state interface configured)
  speed_scaling_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/speed_scaling_input", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr msg) {
      rt_speed_scaling_.writeFromNonRT(msg->data);
    });

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
  speed_scaling_state_interface_.clear();

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

  // Optionally claim the speed scaling state interface
  if (!speed_scaling_state_interface_name_.empty())
  {
    const auto slash = speed_scaling_state_interface_name_.find('/');
    const std::string prefix = speed_scaling_state_interface_name_.substr(0, slash);
    const std::string iface  = speed_scaling_state_interface_name_.substr(slash + 1);
    auto it = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const hardware_interface::LoanedStateInterface & si) {
        return si.get_prefix_name() == prefix && si.get_interface_name() == iface;
      });
    if (it != state_interfaces_.end())
    {
      speed_scaling_state_interface_.emplace_back(std::ref(*it));
      RCLCPP_INFO(get_node()->get_logger(), "Speed scaling state interface claimed");
    }
    else
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Speed scaling state interface '%s' not found, falling back to topic",
        speed_scaling_state_interface_name_.c_str());
    }
  }

  // Initialize Ruckig with current joint positions and pre-seed command
  // interfaces so write() sends the current position even before the first
  // update() cycle (ur_robot_driver does not guard against NaN in write())
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
  speed_scaling_state_interface_.clear();

  trajectory_initialized_ = false;
  new_command_available_ = false;
  rt_command_ptr_.writeFromNonRT(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ConstrainedPositionController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // ---- speed scaling -------------------------------------------------------
  double raw_scaling = 1.0;
  if (!speed_scaling_state_interface_.empty())
  {
    raw_scaling = speed_scaling_state_interface_[0].get().get_value();
  }
  else
  {
    const double * ptr = rt_speed_scaling_.readFromRT();
    if (ptr) { raw_scaling = *ptr; }
  }

  // Record the raw value once, so the unit convention of this hardware is on
  // the record instead of being inferred. The UR state interface is documented
  // as a fraction, but the matching broadcaster topic reports a percentage, and
  // a factor that silently multiplies the kinematic limits is not something to
  // take on trust.
  if (!scaling_logged_ && std::isfinite(raw_scaling) && raw_scaling > 1e-6)
  {
    scaling_logged_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Speed scaling raw value from '%s' is %.6f (values above 1.5 are treated "
      "as a percentage and divided by 100)",
      speed_scaling_state_interface_name_.c_str(), raw_scaling);
  }

  // Normalise to a fraction. A scaling factor is an input that multiplies the
  // kinematic limits, so it is validated like any other untrusted input: a
  // percentage arriving here unnoticed would inflate max_velocity 100x.
  double scaling = std::isfinite(raw_scaling) ? raw_scaling : 0.0;
  if (scaling > 1.5) { scaling /= 100.0; }
  scaling = std::min(std::max(scaling, 0.0), 1.0);

  // Essentially zero (safeguard stop, e-stop, program not running): freeze the
  // trajectory for any duration. The command interfaces keep their last value.
  if (scaling < 1e-6)
  {
    return controller_interface::return_type::OK;
  }

  // ---- new target ----------------------------------------------------------
  auto current_command = rt_command_ptr_.readFromRT();
  if (current_command && *current_command &&
      (*current_command)->data.size() == joint_names_.size())
  {
    ruckig_input_->target_position = (*current_command)->data;
    std::fill(ruckig_input_->target_velocity.begin(),
              ruckig_input_->target_velocity.end(), 0.0);
    std::fill(ruckig_input_->target_acceleration.begin(),
              ruckig_input_->target_acceleration.end(), 0.0);
    rt_command_ptr_.writeFromNonRT(nullptr);

    // The target is validated upstream, so a clamp here means the upstream
    // guarantee has failed. Say so rather than absorbing it silently.
    for (size_t i = 0; i < position_min_.size(); ++i)
    {
      const double t = ruckig_input_->target_position[i];
      const double c = std::min(std::max(t, position_min_[i]), position_max_[i]);
      if (c != t)
      {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Commanded target %.4f for '%s' is outside the window [%.4f, %.4f] "
          "-- clamped to %.4f",
          t, joint_names_[i].c_str(), position_min_[i], position_max_[i], c);
        ruckig_input_->target_position[i] = c;
      }
    }
  }

  // ---- TIME scaling, not limit scaling -------------------------------------
  // A speed override means "traverse the same path more slowly". That is a
  // change of clock, so we change the clock: the trajectory is planned once
  // with the nominal limits and we advance delta_time*s of trajectory time per
  // real cycle.
  //
  // The previous implementation instead rescaled the limits every cycle
  // (vel*s, acc*s^2, jerk*s^3). That is the correct transform for planning a
  // NEW trajectory, but it was applied to one already in flight whose carried
  // over current_velocity/current_acceleration had been produced under the
  // previous cycle's limits. Every change of the override therefore handed
  // Ruckig a state inconsistent with the constraints it was being asked to
  // respect, and s^2/s^3 meant a small drop in the override collapsed the
  // deceleration authority far faster than the velocity it had to arrest.
  // Scaling the clock has no such failure mode: s only decides how far along
  // the same profile we step, so the override can move freely, and s -> 0
  // becomes a natural standstill rather than a special case.
  ruckig_->delta_time = control_cycle_time_ * scaling;

  // ---- solve ---------------------------------------------------------------
  ruckig::Result result;
  try
  {
    result = ruckig_->update(*ruckig_input_, *ruckig_output_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Ruckig update threw: %s -- holding last command", e.what());
    return controller_interface::return_type::OK;
  }

  // Check BEFORE writing. The previous implementation wrote new_position to the
  // hardware and only then inspected the result, so a failed solve still put
  // its output on the wire and pass_to_input fed it back as the next cycle's
  // state -- a loop with nothing to arrest it.
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Ruckig returned %d -- holding last command instead of writing its output",
      static_cast<int>(result));
    return controller_interface::return_type::OK;
  }

  // ---- refuse to run while the arm is outside the window -------------------
  // If the arm is parked outside the window, clamping the setpoint into it
  // would write a position metres of joint travel away from where the machine
  // is, and the robot would take that as a command to get there at once. That
  // is precisely how a stopped-at-the-limit arm gets asked to make a full turn.
  // So we hold instead, keep re-seeding from the measured position, and start
  // tracking only once it has been jogged back inside.
  for (size_t i = 0; i < position_min_.size(); ++i)
  {
    const double actual = joint_position_state_interfaces_[i].get().get_value();
    if (actual < position_min_[i] || actual > position_max_[i])
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "HOLDING: '%s' is at %.4f rad (%.1f deg), outside its window "
        "[%.4f, %.4f]. Jog it back inside; no command will be written.",
        joint_names_[i].c_str(), actual, actual * 180.0 / M_PI,
        position_min_[i], position_max_[i]);
      for (size_t k = 0; k < joint_names_.size(); ++k)
      {
        ruckig_input_->current_position[k] =
          joint_position_state_interfaces_[k].get().get_value();
        ruckig_input_->current_velocity[k] = 0.0;
        ruckig_input_->current_acceleration[k] = 0.0;
      }
      return controller_interface::return_type::OK;
    }
  }

  // ---- bound the commanded trajectory to the machine -----------------------
  // Ruckig integrates from its own output, so without this the commanded
  // position is free to run away from a robot that cannot keep up -- and under
  // a speed override the robot deliberately cannot keep up. Re-seeding from the
  // measured position when the error grows keeps the setpoint attached to the
  // arm; the trajectory simply replans from where the machine actually is.
  double worst = 0.0;
  size_t worst_joint = 0;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const double actual = joint_position_state_interfaces_[i].get().get_value();
    const double err = std::abs(ruckig_output_->new_position[i] - actual);
    if (err > worst) { worst = err; worst_joint = i; }
  }

  if (worst > max_tracking_error_)
  {
    const double actual_worst =
      joint_position_state_interfaces_[worst_joint].get().get_value();
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 500,
      "Tracking error %.4f rad on '%s' (commanded %.4f, actual %.4f, target "
      "%.4f, scaling %.3f) exceeds %.4f -- re-seeding trajectory from measured "
      "position",
      worst, joint_names_[worst_joint].c_str(),
      ruckig_output_->new_position[worst_joint], actual_worst,
      ruckig_input_->target_position[worst_joint], scaling, max_tracking_error_);

    // Inline, allocation-free re-seed (reset_trajectory_state allocates).
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      ruckig_input_->current_position[i] =
        joint_position_state_interfaces_[i].get().get_value();
      ruckig_input_->current_velocity[i] = 0.0;
      ruckig_input_->current_acceleration[i] = 0.0;
    }
    return controller_interface::return_type::OK;
  }

  // ---- last-ditch window enforcement ---------------------------------------
  // Applied to new_position BEFORE pass_to_input, so a clamp also bounds the
  // state Ruckig carries into the next cycle and it cannot keep integrating
  // past the window. Reaching here means Ruckig overshot a valid target, which
  // is the failure mode that drove shoulder_pan to its -2pi stop, so it is an
  // error even though we have contained it.
  for (size_t i = 0; i < position_min_.size(); ++i)
  {
    const double v = ruckig_output_->new_position[i];
    const double c = std::min(std::max(v, position_min_[i]), position_max_[i]);
    if (c != v)
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 500,
        "WINDOW VIOLATION: Ruckig produced %.4f for '%s', outside [%.4f, %.4f] "
        "(target %.4f, scaling %.3f) -- clamped to %.4f",
        v, joint_names_[i].c_str(), position_min_[i], position_max_[i],
        ruckig_input_->target_position[i], scaling, c);
      ruckig_output_->new_position[i] = c;
    }
  }

  // ---- write ---------------------------------------------------------------
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_position_command_interfaces_[i].get().set_value(
      ruckig_output_->new_position[i]);
  }

  ruckig_output_->pass_to_input(*ruckig_input_);

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

bool ConstrainedPositionController::reset_trajectory_state(bool preserve_target)
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

    // Set target to current position (hold) unless caller wants to preserve
    // the existing target (e.g. during safeguard stop, so motion resumes toward
    // the last commanded position once the robot is released)
    if (!preserve_target)
    {
      ruckig_input_->target_position = current_positions;
      ruckig_input_->target_velocity = std::vector<double>(joint_names_.size(), 0.0);
      ruckig_input_->target_acceleration = std::vector<double>(joint_names_.size(), 0.0);
    }

    // Pre-seed command interfaces with current positions so write() sends
    // a safe value even before the first update() cycle
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_position_command_interfaces_[i].get().set_value(current_positions[i]);
    }

    trajectory_initialized_ = true;

    if (!preserve_target)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory state reset to current position");
    }
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