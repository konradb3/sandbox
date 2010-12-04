
#include <ocl/Component.hpp>

#include "ATI6284.h"

void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out)
{
  out.force.x = in[0];
  out.force.y = in[1];
  out.force.z = in[2];

  out.torque.x = in[3];
  out.torque.y = in[4];
  out.torque.z = in[5];
}

ATI6284::ATI6284(const std::string &name) : RTT::TaskContext(name, PreOperational), wrench_port_("wrench"), device_prop_("device", "DAQ device to use", "/dev/comedi0"), offset_prop_("offset", "sensor zero offset", KDL::Wrench::Zero())
{
  this->addPort(wrench_port_);
  this->addProperty(device_prop_);
  this->addProperty(offset_prop_);
}

bool ATI6284::configureHook()
{
    return initSensor();
}

bool ATI6284::startHook()
{
  readData();

  for(int i = 0; i < 6; i++)
    bias_[i] = voltage_ADC_[i];

  return true;
}

void ATI6284::updateHook()
{
  geometry_msgs::Wrench wrenchMsg;

  readData();
  voltage2FT();

  wrench_ -= offset_prop_.value();

  WrenchKDLToMsg(wrench_, wrenchMsg);
  wrench_port_.write(wrenchMsg);
}

void ATI6284::stopHook()
{

}

bool ATI6284::initSensor()
{
  device_ = comedi_open(device_prop_.value().c_str());
  if(!device_)
    return false;

  if(comedi_apply_calibration (device_, 0, 0, 0, 0, NULL) != 0)
    return false;

  comedi_get_hardcal_converter(device_, 0, 0, 0, COMEDI_TO_PHYSICAL, &calib_ADC_);
  return true;
}

void ATI6284::readData()
{
  for(int i = 0; i < 6; i++)
  {
	  comedi_data_read(device_, 0, i, 0, AREF_DIFF, &raw_ADC_[i]);
  }

  for(int i = 0; i < 6; i++)
  {
    voltage_ADC_[i] = comedi_to_physical(raw_ADC_[i], &calib_ADC_);
  }
}

void ATI6284::voltage2FT()
{
  SetToZero(wrench_);

  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      wrench_[i] += (voltage_ADC_[j] - bias_[i]) * conversion_matrix[i][j];
    }

    wrench_[i] /= conversion_scale[i];
  }
}

ORO_CREATE_COMPONENT( ATI6284 )
