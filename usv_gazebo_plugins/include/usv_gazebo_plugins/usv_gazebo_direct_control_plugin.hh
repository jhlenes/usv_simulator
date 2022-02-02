#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <memory>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include "ros/callback_queue.h"

namespace gazebo
{
  /**
  * @brief Plugin class to implement direct control of the USV model pose and velocity.
  *
  * The plugin expects a "gazebo_usv_direct_control_plugin/usv_data" topic with a
  * "std_msgs::Float64MultiArray" containing 7 values: x, y, z, roll, pitch, yaw, forward speed.
  *
  * These values are used to set the model position, attitude and speed directly.
  */
  class UsvDirectControlPlugin : public ModelPlugin
  {
    public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    void OnUpdate();
    void OnOtterDataReceived(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void QueueThread();

  private:
    // Pointer to the model
    physics::ModelPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    // The ROS node handler used for communications.
    std::unique_ptr<ros::NodeHandle> rosnode;
    // A ROS subscriber
    ros::Subscriber rossub;
    // A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;
    // A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    // The USV pose
    ignition::math::Pose3d pose;
    // The USV velocity
    ignition::math::Vector3d linearVelocity;
  };
}
