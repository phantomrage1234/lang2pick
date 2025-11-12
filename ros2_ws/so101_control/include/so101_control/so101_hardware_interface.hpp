#ifndef SO101_HARDWARE_INTERFACE_HPP
#define SO101_HARDWARE_INTERFACE_HPP

#include <hardware_interface/system_interface.hpp>

using hardware_interface::CallbackReturn;

namespace so101_control {

class so101_hardware_interface final
    : public hardware_interface::SystemInterface {
private:
  /* data */
public:
  so101_hardware_interface();
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  CallbackReturn on_configure(const hardware_interface::HardwareInfo &info);
  CallbackReturn on_activate(const hardware_interface::HardwareInfo &info);
  CallbackReturn on_deactivate(const hardware_interface::HardwareInfo &info);
};

} // namespace so101_control

#endif