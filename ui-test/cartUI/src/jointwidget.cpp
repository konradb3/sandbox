#include <QtGui>

#include "jointwidget.h"
#include <urdf/joint.h>


JointWidget::JointWidget(ros::NodeHandle &nh, QWidget *parent) :
    QWidget(parent), n(nh), traj_client(NULL)
{
  QGridLayout *mainLayout = new QGridLayout;

  QGroupBox *msrBox = new QGroupBox("msr");
  QGroupBox *setBox = new QGroupBox("setpoints");
  QGroupBox *nameBox = new QGroupBox("joint name");

  mainLayout->addWidget(nameBox, 0, 0);
  mainLayout->addWidget(msrBox, 0, 1);
  mainLayout->addWidget(setBox, 0, 2);

  msrBoxL = new QVBoxLayout(msrBox);
  setBoxL = new QVBoxLayout(setBox);
  nameBoxL = new QVBoxLayout(nameBox);

  moveB = new QPushButton("Move");
  moveB->setDisabled(true);
  time = new QDoubleSpinBox();
  time->setMinimum(1.0);
  QLabel *timeL = new QLabel("time:");
  mainLayout->addWidget(timeL, 1, 0);
  mainLayout->addWidget(time, 1, 1);
  mainLayout->addWidget(moveB, 1, 2);

  QPushButton *propB = new QPushButton("Properties");
  mainLayout->addWidget(propB, 2, 2);

  bar = new QStatusBar(this);
  bar->showMessage("Not connected");
  mainLayout->addWidget(bar, 2, 0, 1, 2);

  pDialog = new PropertyDialog();
  pDialog->setWindowModality(Qt::ApplicationModal);

  setLayout(mainLayout);
  setWindowTitle(tr("Joint Trajectory Action GUI"));

  connect(moveB, SIGNAL(clicked()), this, SLOT(move()));
  connect(propB, SIGNAL(clicked()), this, SLOT(properties()));

  robot_model.initParam("/robot_description");
}

void JointWidget::addJoint(const QString &name, double lLimit, double uLimit)
{
  QLabel *nameLabel = new QLabel(name);
  nameBoxL->addWidget(nameLabel);
  nameLabels.insert(name, nameLabel);

  QLCDNumber *msrLabel = new QLCDNumber(6);
  msrLabel->setSegmentStyle(QLCDNumber::Flat);
  msrBoxL->addWidget(msrLabel);
  msrLabels.insert(name, msrLabel);

  QDoubleSpinBox *dsdsd = new QDoubleSpinBox();
  dsdsd->setMinimum(lLimit);
  dsdsd->setMaximum(uLimit);
  setBoxL->addWidget(dsdsd);
  setButtons.insert(name, dsdsd);
}

void JointWidget::init()
{
//  addJoint(QString("joint1"));
// addJoint(QString("joint2"));
//  addJoint(QString("joint3"));
//  addJoint(QString("joint4"));
//  addJoint(QString("joint5"));
//  addJoint(QString("joint6"));

 // jnt = n.subscribe("/joint_states", 1, &JointWidget::jointStateCallback, this);

 // traj_client = new TrajClient("irp6p_controller/joint_trajectory_action", true);

 // if(traj_client->waitForServer(ros::Duration(10.0)))
 //   moveB->setDisabled(false);

}

void JointWidget::move()
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.points.resize(1);
  for(QHash<QString, QDoubleSpinBox*>::iterator i = setButtons.begin(); i != setButtons.end(); i++)
  {
    goal.trajectory.joint_names.push_back(i.key().toStdString());
    goal.trajectory.points[0].positions.push_back((*i)->value());
    goal.trajectory.points[0].velocities.push_back(0.0);
  }

  goal.trajectory.points[0].time_from_start = ros::Duration(time->value());
  goal.trajectory.header.stamp = ros::Time::now();

  traj_client->sendGoal(goal, boost::bind(&JointWidget::trajectoryDoneCallback, this, _1, _2));
  moveB->setDisabled(true);
  bar->showMessage("Motion in progress ...");
}

void JointWidget::properties()
{
  pDialog->show();

  if(pDialog->exec() == QDialog::Accepted)
  {

    XmlRpc::XmlRpcValue joints;
    if(n.getParam(pDialog->getTrajectoryAction().toStdString() + "/joints", joints))
    {
      if(joints.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        cleanJointList();
        for (int32_t i = 0; i < joints.size(); ++i)
        {
          boost::shared_ptr<const urdf::Joint> j = robot_model.getJoint(static_cast<std::string>(joints[i]).c_str());
          if(j->type == urdf::Joint::CONTINUOUS)
          {
            addJoint(QString(static_cast<std::string>(joints[i]).c_str()), -3.14, 3.14);
          } else
          {
            addJoint(QString(static_cast<std::string>(joints[i]).c_str()), j->limits->lower, j->limits->upper);
          }
        }
      }
      jnt.shutdown();

      jnt = n.subscribe(pDialog->getJointTopic().toStdString(), 1, &JointWidget::jointStateCallback, this);

      ros::Duration(0.5).sleep();

      if(jnt.getNumPublishers() > 0)
      {
        bar->showMessage("Joint State connected");
        delete traj_client;
        traj_client = new TrajClient(pDialog->getTrajectoryAction().toStdString() + "/joint_trajectory_action", true);

        if(traj_client->waitForServer(ros::Duration(2.0)))
        {
          bar->showMessage("Ready");
          moveB->setDisabled(false);
        } else
        {
          moveB->setDisabled(true);
        }
      } else
      {
        bar->showMessage("Not connected");
        moveB->setDisabled(true);
      }
    }
  }
}

void JointWidget::trajectoryDoneCallback(const actionlib::SimpleClientGoalState& state, const pr2_controllers_msgs::JointTrajectoryResult::ConstPtr& result)
{
  moveB->setDisabled(false);
//  bar->showMessage("Ready");
}

void JointWidget::cleanJointList()
{
  for(QHash<QString, QDoubleSpinBox*>::iterator i = setButtons.begin(); i != setButtons.end(); i++)
  {
    msrLabels[i.key()]->close();
    delete msrLabels[i.key()];
    setButtons[i.key()]->close();
    delete setButtons[i.key()];
    nameLabels[i.key()]->close();
    delete nameLabels[i.key()];
  }
  msrLabels.clear();
  setButtons.clear();
  nameLabels.clear();

}

void JointWidget::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(unsigned int i = 0; i < msg->name.size(); i++)
  {
    if(msrLabels.contains(QString(msg->name[i].c_str())))
    {
      msrLabels[QString(msg->name[i].c_str())]->display(msg->position[i]);
    }
  }
}
