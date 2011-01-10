#include <QtGui/QApplication>

#include <ros/ros.h>

#include "cartesianwidget.h"
#include "rosthread.h"

int main(int argc, char *argv[])
{
  ROSThread t;
  ros::init(argc, argv, "JointUI");

  ros::NodeHandle nh;

  QApplication a(argc, argv);
  CartesianWidget jointW(nh);

  jointW.show();

  t.start();
  a.exec();

  ros::shutdown();
  t.wait();
}
