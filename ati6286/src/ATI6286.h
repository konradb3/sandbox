#ifndef ATI6286_H
#define ATI6286_H

#include <string>
#include <comedilib.h>

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Property.hpp>

#include <kdl/frames.hpp>

#include "geometry_msgs/Wrench.h"

const double conversion_scale[6] = {
  4.5511972116989, 4.5511972116989, 1.41244051397552, 84.8843245576086, 84.8843245576086, 80.9472037525247 };


const double conversion_matrix[6][6] = {
  {-0.40709,  -0.27318,   0.34868, -33.58156,  -0.32609,  33.54162},
  { 0.35472,  38.22730,  -0.41173, -19.49156,   0.49550, -19.15271},
  {18.72635,  -0.59676,  19.27843,  -0.56931,  18.69352,  -0.67633},
  {-0.40836,  -0.95908, -33.37957,   1.38537,  32.52522,  -0.51156},
  {37.13715,  -1.02875, -20.00474,  -0.27959, -19.34135,   1.42577},
  {-0.15775, -18.16831,  -0.00133, -18.78961,   0.31895, -18.38586}
};

class ATI6286 : public RTT::TaskContext
{
public:
    ATI6286(const std::string &name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();

protected:
    RTT::OutputPort<geometry_msgs::Wrench> wrench_port_;
private:
    comedi_t *device_;
    lsampl_t raw_ADC_[6];

    double voltage_ADC_[6];
    double bias_[6];
    comedi_polynomial_t calib_ADC_;

    KDL::Wrench wrench_;
    KDL::Frame force_tool_;

    bool initSensor();
    void readData();
    void voltage2FT();


};

#endif // ATI6286_H
