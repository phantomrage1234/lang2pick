#include "so101_control/so101_hardware_interface.hpp"

so101_control::so101_hardware_interface::so101_hardware_interface()
{
}

hardware_interface::CallbackReturn
so101_control::so101_hardware_interface::on_init(const hardware_interface::HardwareInfo& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  auto it = params.hardware_parameters.find("usb_port");
  if (it == params.hardware_parameters.end())
  {
    RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "USB port not specified in hardware parameters.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  usb_port_ = it->second;
  auto baud_it = params.hardware_parameters.find("baud_rate");
  if (baud_it == params.hardware_parameters.end())
  {
    RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Baud rate not specified in hardware parameters.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  baud_rate_ = std::stoi(baud_it->second);
  num_joints_ = params.joints.size();
  servos_.resize(num_joints_);
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Initializing hardware interface for %d joints.",
              num_joints_);

  num_joints_ = params.joints.size();
  std::unordered_set<std::string> joint_names, state_interface_names, command_interface_names;

  for (int i = 0; i < num_joints_; ++i)
  {
    if (joint_names.find(params.joints[i].name) != joint_names.end())
    {
      RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Joint name %s is not unique.",
                   params.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (params.joints[i].state_interfaces.size() != 5)
    {
      RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Joint %s does not have 5 state interfaces.",
                   params.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    state_interface_names.clear();

    for (const auto& state_interface : params.joints[i].state_interfaces)
    {
      if (state_interface_names.find(state_interface.name) != state_interface_names.end() ||
          (state_interface.name != "position" && state_interface.name != "velocity" &&
           state_interface.name != "torque" && state_interface.name != "temperature" &&
           state_interface.name != "voltage"))
      {
        RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"),
                     "Joint %s has duplicate/unknown state interface %s.", params.joints[i].name.c_str(),
                     state_interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      state_interface_names.insert(state_interface.name);
    }

    if (params.joints[i].command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Joint %s does not have 2 command interfaces.",
                   params.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    command_interface_names.clear();
    for (const auto& command_interface : params.joints[i].command_interfaces)
    {
      if (command_interface_names.find(command_interface.name) != command_interface_names.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Joint %s has duplicate command interface %s.",
                     params.joints[i].name.c_str(), command_interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      command_interface_names.insert(command_interface.name);
    }
    auto id_it = params.joints[i].parameters.find("id");
    if (id_it == params.joints[i].parameters.end())
    {
      RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Joint %s is missing parameter 'id'.",
                   params.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    int id = std::stoi(params.joints[i].parameters.at("id"));
    servos_[i] = so101_control::Servo(id);
    auto offset_it = params.joints[i].parameters.find("position_offset");
    if (offset_it != params.joints[i].parameters.end())
    {
      servos_[i].position_offset = std::stod(offset_it->second);
    }
    servos_[i].cmd_id = params.joints[i].parameters.at("type") == "position" ? 0.0 : 1.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
so101_control::so101_hardware_interface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!motor_driver_.begin(baud_rate_, usb_port_.c_str()))
  {
    RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Failed to open motor driver on port: %s",
                 usb_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Successfully connected to motor driver on port: %s",
              usb_port_.c_str());

  // Ping each servo to ensure they are connected
  for (int i = 0; i < num_joints_; ++i)
  {
    if (motor_driver_.Ping(servos_[i].id) == -1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("so101_hardware_interface"), "Failed to ping motor ID %d.", servos_[i].id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Successfully pinged all motor IDs.");

  // setting up each servo
  for (int i = 0; i < num_joints_; ++i)
  {
    motor_driver_.Mode(servos_[i].id, 0);  // position control mode
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
so101_control::so101_hardware_interface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  motor_driver_.end();
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Motor driver connection closed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
so101_control::so101_hardware_interface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Activating hardware interface for %d joints.",
              num_joints_);

  for (int i = 0; i < num_joints_; ++i)
  {
    double initial_pose = servos_[i].position_state;
    servos_[i].cmd_position = initial_pose;
    servos_[i].cmd_velocity = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn so101_control::so101_hardware_interface::update_hardware_status(int id)
{
  if (motor_driver_.FeedBack(id) == -1)
  {
    RCLCPP_ERROR(rclcpp::get_logger("so101_hardware_interface"), "Failed to get feedback from motor ID %d.", id);
    return hardware_interface::CallbackReturn::ERROR;
  }
  else
  {
    servos_[id - 1].position_state = motor_driver_.ReadPos(id) * 2 * M_PI / steps_per_rev_;
    servos_[id - 1].velocity_state = motor_driver_.ReadSpeed(id) * 2 * M_PI / steps_per_rev_;
    servos_[id - 1].torque_state = motor_driver_.ReadLoad(id);
    servos_[id - 1].voltage_state = motor_driver_.ReadVoltage(id);
    servos_[id - 1].temperature_state = motor_driver_.ReadTemper(id);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
so101_control::so101_hardware_interface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Deactivating hardware interface for %d joints.",
              num_joints_);
  for (int i = 0; i < num_joints_; ++i)
  {
    motor_driver_.EnableTorque(i + 1, 0);
  }
  this->write(rclcpp::Time(), rclcpp::Duration(0, 0));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
so101_control::so101_hardware_interface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Shutting down...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
so101_control::so101_hardware_interface::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("so101_hardware_interface"), "Error...");
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> so101_control::so101_hardware_interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (int i = 0; i < num_joints_; ++i)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "position", &servos_[i].position_state));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "velocity", &servos_[i].velocity_state));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "torque", &servos_[i].torque_state));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "temperature", &servos_[i].temperature_state));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, "voltage", &servos_[i].voltage_state));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> so101_control::so101_hardware_interface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (int i = 0; i < num_joints_; ++i)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "position", &servos_[i].cmd_position));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "velocity", &servos_[i].cmd_velocity));
  }
  return command_interfaces;
}

hardware_interface::return_type so101_control::so101_hardware_interface::read(const rclcpp::Time& /*time*/,
                                                                              const rclcpp::Duration& /*period*/)
{
  for (int i = 1; i <= num_joints_; ++i)
  {
    if (update_hardware_status(i) == hardware_interface::CallbackReturn::ERROR)
    {
      RCLCPP_ERROR(rclcpp::get_logger("so101_hardware_interface"),
                   "Failed to get feedback from motor ID %d during read.", servos_[i].id);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type so101_control::so101_hardware_interface::write(const rclcpp::Time& /*time*/,
                                                                               const rclcpp::Duration& /*period*/)
{
  for (int i = 0; i < num_joints_; ++i)
  {
    int id = servos_[i].id;
    s16 position = static_cast<s16>(servos_[i].cmd_position);
    s16 speed = static_cast<s16>(servos_[i].cmd_velocity);
    motor_driver_.WritePosEx(id, position, speed, 0);
  }
  return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(so101_control::so101_hardware_interface, hardware_interface::SystemInterface)