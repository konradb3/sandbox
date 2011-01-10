#include "rosthread.h"

ROSThread::ROSThread(QObject *parent) :
    QThread(parent)
{
}

void ROSThread::run()
{
  ros::spin();
}
