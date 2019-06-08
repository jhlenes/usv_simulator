#ifndef OTTER_CONTROL_H
#define OTTER_CONTROL_H

#include <ros/ros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <usv_msgs/SpeedCourse.h>

#include <eigen3/Eigen/Core>

class OtterController
{
public:
  OtterController();

private:
  double calculateSurgeForce(double deltaTime, double u);
  double calculateYawMoment(double deltaTime, double psi, double r);
  Eigen::Vector2d thrustAllocation(Eigen::Vector3d tau_d);
  void inputCallback(const usv_msgs::SpeedCourse& msg);
  void speedCallback(const geometry_msgs::Vector3Stamped& msg);
  void imuCallback(const sensor_msgs::Imu& msg);
  double getYaw();

  // Heading controller
  double Kp_psi = 4;
  double Ki_psi = 1.0;
  double Kd_psi = 4;
  double mass_psi = 10.0 - 1.0; // Iz - nDotR
  double damp_psi = 20.0; // nR

  // Speed controller
  double Kp_u = 2.0;
  double Ki_u = 1.0;
  double mass_u = 29 - 5.0; // m - xDotU: 29 - 5
  double damp_u = 20.0; // xU

  // Sensor data
  double u = 0.0;
  double psi = 0.0;
  double r = 0.0;

  // Desired values
  double u_d = 0.0;
  double psi_d = 0.0;

  // Thruster configuration matrix
  Eigen::MatrixXd T;

  // Publishers for thruster commands
  ros::Publisher m_leftPub;
  ros::Publisher m_rightPub;
};

#endif
