#include <tf/transform_listener.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionIK.h>

#include "cartesianwidget.h"
#include "ui_cartesianwidget.h"

Q_DECLARE_METATYPE(tf::StampedTransform);
Q_DECLARE_METATYPE(sensor_msgs::JointState);

CartesianWidget::CartesianWidget(ros::NodeHandle &nh, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CartesianWidget),
    n(nh),
    traj_client(NULL)
{
    qRegisterMetaType<tf::StampedTransform>();
    qRegisterMetaType<sensor_msgs::JointState>();

    ui->setupUi(this);
    prop = new PropertyDialog();
    listener = new tf::TransformListener(n);
    listener->addTransformsChangedListener(boost::bind(&CartesianWidget::transform, this));

    connect(ui->propButton, SIGNAL(clicked()), this, SLOT(properties()));
    connect(ui->moveButton, SIGNAL(clicked()), this, SLOT(move()));
    connect(ui->copyJointsButton, SIGNAL(clicked()), this, SLOT(copyJoints()));
    connect(ui->copyCartButton, SIGNAL(clicked()), this, SLOT(copyCart()));

    connect(this, SIGNAL(jointStateSig(sensor_msgs::JointState)), this, SLOT(jointStateSl(sensor_msgs::JointState)), Qt::QueuedConnection);
    connect(this, SIGNAL(transformSig(tf::StampedTransform)), this, SLOT(transformSl(tf::StampedTransform)), Qt::QueuedConnection);
    
}

CartesianWidget::~CartesianWidget()
{
    delete ui;
}

void CartesianWidget::move()
{
  if(ui->tabWidget->currentWidget()->objectName() == "cartesianTab")
  {

    tf::Stamped<tf::Pose> transform, transform2;
    transform.frame_id_ = ui->controlFrameBox->currentText().toStdString();
    transform.setOrigin(btVector3(ui->posXBox->value(), ui->posYBox->value(), ui->posZBox->value()));
    transform.setRotation(tf::createQuaternionFromRPY(ui->rotRBox->value(), ui->rotPBox->value(), ui->rotYBox->value()));

    listener->transformPose(root_name, transform, transform2);

    kinematics_msgs::GetPositionIK::Request  gpik_req;
    kinematics_msgs::GetPositionIK::Response gpik_res;
    gpik_req.timeout = ros::Duration(5.0);
    gpik_req.ik_request.ik_link_name = tip_name;

    poseStampedTFToMsg(transform2, gpik_req.ik_request.pose_stamped);

    // define the service messages
    kinematics_msgs::GetKinematicSolverInfo::Request request;
    kinematics_msgs::GetKinematicSolverInfo::Response response;

    if(query_client.call(request,response))
    {
      for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
      {
        ROS_DEBUG("Joint: %d %s",i,response.kinematic_solver_info.joint_names[i].c_str());
      }
    }

    gpik_req.ik_request.ik_seed_state.joint_state.position.resize(response.kinematic_solver_info.joint_names.size());
    gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names;
    for(unsigned int i=0; i< response.kinematic_solver_info.joint_names.size(); i++)
    {
      gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0;
    }


    if(ik_client.call(gpik_req, gpik_res))
    {
      if(gpik_res.error_code.val == gpik_res.error_code.SUCCESS)
      {
        for(unsigned int i=0; i < gpik_res.solution.joint_state.name.size(); i ++)
          ROS_INFO("Joint: %s %f",gpik_res.solution.joint_state.name[i].c_str(),gpik_res.solution.joint_state.position[i]);
        moveJoints(gpik_res.solution.joint_state.name, gpik_res.solution.joint_state.position, ui->timeBox->value());
      } else
        ROS_ERROR("Inverse kinematics failed");
    }
    else
      ROS_ERROR("Inverse kinematics service call failed");
  } else
  {
    std::vector<std::string> jointNames;
    std::vector<double> jointSetpoints;
    for(QHash<QString, QDoubleSpinBox*>::iterator i = setButtons.begin(); i != setButtons.end(); i++)
    {
      jointNames.push_back(i.key().toStdString());
      jointSetpoints.push_back((*i)->value());
    }
    moveJoints(jointNames, jointSetpoints, ui->timeBox->value());
  }
}

void CartesianWidget::properties()
{
  prop->show();

  if(prop->exec() == QDialog::Accepted)
  {

    bool ikOK = ikInit(prop->getKinematicName());

    bool jointOK = jointStateInit(prop->getJointTopicName());

    bool controllerOK = jointActionInit(prop->getControllerName());

    if(jointOK && controllerOK)
    {
      ui->moveButton->setDisabled(false);
      if(ikOK)
      {
        ui->cartesianTab->setDisabled(false);
      }else
      {
        ui->cartesianTab->setDisabled(true);
      }
    } else
    {
      ui->moveButton->setDisabled(true);
    }

  }
}

bool CartesianWidget::ikInit(QString ikNode)
{
  std::vector<std::string> frames;
  if(n.getParam(ikNode.toStdString() + "/root_name", root_name) &&
     n.getParam(ikNode.toStdString() + "/tip_name", tip_name) )
  {
    listener->waitForTransform(tip_name, root_name, ros::Time::now(), ros::Duration(2.0));
    listener->getFrameStrings(frames);

    for(unsigned int i = 0; i < frames.size(); i++)
    {
      ui->controlFrameBox->addItem(QString(frames[i].c_str()));
    }
  } else
  {
    return false;
  }

  ik_client = n.serviceClient<kinematics_msgs::GetPositionIK>(ikNode.toStdString() + "/get_ik");
  query_client = n.serviceClient<kinematics_msgs::GetKinematicSolverInfo>(ikNode.toStdString() + "/get_ik_solver_info");

  return true;
}

void CartesianWidget::transform()
{
  tf::StampedTransform transform;
  try{
    listener->lookupTransform(ui->controlFrameBox->currentText().toStdString(), tip_name,
                             ros::Time(0), transform);
    emit transformSig(transform);
  }
  catch (tf::TransformException ex){

  }
}

void CartesianWidget::transformSl(const tf::StampedTransform transform)
{
    ui->posXDisp->display(transform.getOrigin().x());
    ui->posYDisp->display(transform.getOrigin().y());
    ui->posZDisp->display(transform.getOrigin().z());
    double R, P, Y;
    btMatrix3x3(transform.getRotation()).getRPY(R, P, Y);

    ui->rotRDisp->display(R);
    ui->rotPDisp->display(P);
    ui->rotYDisp->display(Y);
}

void CartesianWidget::moveJoints(std::vector<std::string> &jointNames, std::vector<double> &jointPositions, double time)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.points.resize(1);

  goal.trajectory.joint_names = jointNames;
  goal.trajectory.points[0].positions = jointPositions;

  for(unsigned int i = 0; i > jointNames.size(); i++)
  {
    goal.trajectory.points[0].velocities.push_back(0.0);
  }

  goal.trajectory.points[0].time_from_start = ros::Duration(time);
  goal.trajectory.header.stamp = ros::Time::now();

  traj_client->sendGoal(goal);
}

void CartesianWidget::cleanJointList()
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

void CartesianWidget::addJoint(const QString &name, double lLimit, double uLimit)
{
  QLabel *nameLabel = new QLabel(name);
  ui->jointNameLayout->addWidget(nameLabel);
  nameLabels.insert(name, nameLabel);

  QLCDNumber *msrLabel = new QLCDNumber(6);
  msrLabel->setSegmentStyle(QLCDNumber::Flat);
  ui->jointPosLayout->addWidget(msrLabel);
  msrLabels.insert(name, msrLabel);

  QDoubleSpinBox *dsdsd = new QDoubleSpinBox();
  dsdsd->setMinimum(lLimit);
  dsdsd->setMaximum(uLimit);
  ui->jointSetpointLayout->addWidget(dsdsd);
  setButtons.insert(name, dsdsd);
}

bool CartesianWidget::jointActionInit(QString controllerName)
{
  robot_model.initParam("/robot_description");
  XmlRpc::XmlRpcValue joints;
  if(n.getParam(controllerName.toStdString() + "/joints", joints))
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
  } else
  {
    return false;
  }

  delete traj_client;
  traj_client = new TrajClient(n, controllerName.toStdString() + "/joint_trajectory_action", false);

  if(traj_client->waitForServer(ros::Duration(2.0)))
  {
    return true;
  } else
  {
    return false;
  }
}

bool CartesianWidget::jointStateInit(QString topic)
{
  jnt.shutdown();

  jnt = n.subscribe(topic.toStdString(), 1, &CartesianWidget::jointStateCallback, this);
  ros::Duration(0.5).sleep();

  if(jnt.getNumPublishers() > 0)
  {
    return true;
  } else
  {
    return false;
  }
}

void CartesianWidget::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    emit jointStateSig(*msg);
}

void CartesianWidget::jointStateSl(const sensor_msgs::JointState msg)
{
    for(unsigned int i = 0; i < msg.name.size(); i++)
    {
      if(msrLabels.contains(QString(msg.name[i].c_str())))
      {
        msrLabels[QString(msg.name[i].c_str())]->display(msg.position[i]);
      }
    }
}

void CartesianWidget::copyJoints()
{
  for(QHash<QString, QDoubleSpinBox*>::iterator i = setButtons.begin(); i != setButtons.end(); i++)
  {
    (*i)->setValue(msrLabels[i.key()]->value());
  }
}

void CartesianWidget::copyCart()
{
  ui->posXBox->setValue(ui->posXDisp->value());
  ui->posYBox->setValue(ui->posYDisp->value());
  ui->posZBox->setValue(ui->posZDisp->value());

  ui->rotRBox->setValue(ui->rotRDisp->value());
  ui->rotPBox->setValue(ui->rotPDisp->value());
  ui->rotYBox->setValue(ui->rotYDisp->value());
}
