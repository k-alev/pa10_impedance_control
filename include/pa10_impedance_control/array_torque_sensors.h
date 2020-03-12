#ifndef HARDWARE_INTERFACE_ARRAY_TORQUE_SENSORS_INTERFACE_H
#define HARDWARE_INTERFACE_ARRAY_TORQUE_SENSORS_INTERFACE_H
 
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>
 
namespace hardware_interface
{

class ArrayTorqueSensorsHandle
{
public:
  ArrayTorqueSensorsHandle() : name_(""), frame_id_(""), torques_(0) {}

  ArrayTorqueSensorsHandle(const std::string& name,
                          const std::string& frame_id,
                          double* torques)
    : name_(name),
      frame_id_(frame_id),
      torques_(torques)
  {}

  std::string getName()     const {return name_;}
  std::string getFrameId()  const {return frame_id_;}
  const double* getTorques() const {return torques_;}

private:
  std::string name_;
  std::string frame_id_;
  double* torques_;
};

class ArrayTorqueSensorsInterface : public HardwareResourceManager<ArrayTorqueSensorsHandle> {};

}
 
#endif // HARDWARE_INTERFACE_ARRAY_TORQUE_SENSORS_INTERFACE_H
