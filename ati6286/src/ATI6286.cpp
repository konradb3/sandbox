
#include <ocl/Component.hpp>

#include "ATI6286.h"

void WrenchKDLToMsg(const KDL::Wrench &in, geometry_msgs::Wrench &out)
{
  out.force.x = in[0] + 3.6095;
  out.force.y = in[1] - 2.3973;
  out.force.z = in[2] + 6.1467;

  out.torque.x = in[3] - 0.7662;
  out.torque.y = in[4] + 0.4179;
  out.torque.z = in[5] - 0.022;
}

ATI6286::ATI6286(const std::string &name) : RTT::TaskContext(name, PreOperational), wrench_port_("wrench")
{
  this->addPort(wrench_port_);
}

bool ATI6286::configureHook()
{
    return initSensor();
}

bool ATI6286::startHook()
{
  readData();

  for(int i = 0; i < 6; i++)
    bias_[i] = 0.0; //voltage_ADC_[i];

  return true;
}

void ATI6286::updateHook()
{
  geometry_msgs::Wrench wrenchMsg;

  readData();
  voltage2FT();

  wrench_ = force_tool_ * wrench_; // force tool transform

  WrenchKDLToMsg(wrench_, wrenchMsg);
  wrench_port_.write(wrenchMsg);
}

void ATI6286::stopHook()
{

}

bool ATI6286::initSensor()
{
  device_ = comedi_open("/dev/comedi0");
  if(!device_)
    return false;

  if(comedi_apply_calibration (device_, 0, 0, 0, 0, NULL) != 0)
    return false;

  comedi_get_hardcal_converter(device_, 0, 0, 0, COMEDI_TO_PHYSICAL, &calib_ADC_);
  return true;
}

void ATI6286::readData()
{

 for(int i = 0; i < 5; i++)
 {
  comedi_data_read(device_, 0, 0, 0, AREF_DIFF, &raw_ADC_[0]);
  comedi_data_read(device_, 0, 1, 0, AREF_DIFF, &raw_ADC_[1]);
  comedi_data_read(device_, 0, 2, 0, AREF_DIFF, &raw_ADC_[2]);
  comedi_data_read(device_, 0, 3, 0, AREF_DIFF, &raw_ADC_[3]);
  comedi_data_read(device_, 0, 4, 0, AREF_DIFF, &raw_ADC_[4]);
  comedi_data_read(device_, 0, 5, 0, AREF_DIFF, &raw_ADC_[5]);

  for(int i = 0; i < 6; i++)
    voltage_ADC_[i] += comedi_to_physical(raw_ADC_[i], &calib_ADC_);
  
 }

 for(int i = 0; i < 6; i++)
   voltage_ADC_[i] /= 5;
}

void ATI6286::voltage2FT()
{
  double tmp[6];

  SetToZero(wrench_);

  for(int i = 0; i < 6; i++)
  {
    tmp[i] = voltage_ADC_[i] - bias_[i];
    voltage_ADC_[i] = 0.0;
  }

  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      wrench_[i] += tmp[j] * conversion_matrix[i][j];
    }

    wrench_[i] /= conversion_scale[i];
  }
}

ORO_CREATE_COMPONENT( ATI6286 )
