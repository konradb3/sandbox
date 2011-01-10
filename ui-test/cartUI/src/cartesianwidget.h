#ifndef CARTESIANWIDGET_H
#define CARTESIANWIDGET_H

#include <QWidget>
#include <QHash>
#include <QString>

#include <ros/ros.h>
#include <urdf/model.h>

#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include "propertydialog.h"

namespace tf {
  class TransformListener;
}

namespace Ui {
    class CartesianWidget;
}

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class QLabel;
class QDoubleSpinBox;
class QVBoxLayout;
class QPushButton;
class QLCDNumber;
class QStatusBar;

class CartesianWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CartesianWidget(ros::NodeHandle &nh, QWidget *parent = 0);
    ~CartesianWidget();
public slots:
    void move();
    void properties();
    void copyJoints();
    void copyCart();
private:
    //QT
    Ui::CartesianWidget *ui;
    PropertyDialog *prop;

    QHash<QString, QDoubleSpinBox*> setButtons;
    QHash<QString, QLCDNumber*> msrLabels;
    QHash<QString, QLabel*> nameLabels;

    void addJoint(const QString &name, double lLimit, double uLimit);
    void cleanJointList();

    //ROS
    ros::NodeHandle n;
    ros::Subscriber jnt;
    tf::TransformListener *listener;
    TrajClient* traj_client;
    urdf::Model robot_model;
    ros::ServiceClient ik_client;
    ros::ServiceClient query_client;
    std::string tip_name;
    std::string root_name;

    bool jointActionInit(QString controllerName);
    bool jointStateInit(QString topic);
    bool ikInit(QString ikNode);

    void transform();
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    void moveJoints(std::vector<std::string> &jointNames, std::vector<double> &jointPositions, double time);
};

#endif // CARTESIANWIDGET_H
