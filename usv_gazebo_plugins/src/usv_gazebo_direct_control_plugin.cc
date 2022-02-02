#include "usv_gazebo_plugins/usv_gazebo_direct_control_plugin.hh"

#include <ros/ros.h>

using namespace gazebo;

void UsvDirectControlPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Store the pointer to the model
  this->model = _parent;

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_usv_direct_control_plugin",
        ros::init_options::NoSigintHandler);
  }

  // Create ROS node and subscribe to topic
  this->rosnode.reset(new ros::NodeHandle("gazebo_usv_direct_control_plugin"));
  this->rossub = this->rosnode->subscribe("usv_data", 1000, &UsvDirectControlPlugin::OnOtterDataReceived, this);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&UsvDirectControlPlugin::QueueThread, this));

  // Listen to the update event. This event is broadcast every simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvDirectControlPlugin::OnUpdate, this));
}

// Called by the world update start event
void UsvDirectControlPlugin::OnUpdate()
{
  // Set pose and velocities
  this->model->SetWorldPose(this->pose);
  this->model->SetLinearVel(this->linearVelocity);
  this->model->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
}

void UsvDirectControlPlugin::OnOtterDataReceived(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  if (msg->data.size() < 7)
    return;

  // Store pose and velocities
  // x, y, z, roll, pitch, yaw
  this->pose = ignition::math::Pose3d(msg->data.at(0), msg->data.at(1), msg->data.at(2),
                                      msg->data.at(3), msg->data.at(4), msg->data.at(5));
  // velocity forward, velocity starboard, velocity upwards
  this->linearVelocity = ignition::math::Vector3d(msg->data.at(6), 0.0, 0.0);
}

void UsvDirectControlPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosnode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UsvDirectControlPlugin)
