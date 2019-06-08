#include <otter_control/controller.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/QR>

#include <cmath>

OtterController::OtterController() : T(3, 2)
{
  ros::NodeHandle nh;

  m_leftPub = nh.advertise<std_msgs::Float32>("left_thrust_cmd", 10);
  m_rightPub = nh.advertise<std_msgs::Float32>("right_thrust_cmd", 10);

  ros::Subscriber sub = nh.subscribe("speed_heading", 1000, &OtterController::inputCallback, this);

  ros::Subscriber subSpeed = nh.subscribe("gps/vel", 1000, &OtterController::speedCallback, this);

  ros::Subscriber subImu = nh.subscribe("imu/data", 1000, &OtterController::imuCallback, this);

  // Initialize thruster configuration matrix
  T << 50, 50, 0, 0, -0.39 * 50, 0.39 * 50;

  double frequency = 10.0;
  double deltaTime = 1.0 / frequency;
  ros::Rate rate(frequency);
  while (nh.ok()) {

    // Get SLAM heading
    double psi_slam = getYaw();

    // Speed controller
    auto tauSurge = calculateSurgeForce(deltaTime, u);

    // Heading controller
    auto tauYaw = calculateYawMoment(deltaTime, psi_slam, r);

    // Thrust allocation
    auto cmdThrust = thrustAllocation({tauSurge, 0.0, tauYaw});

    // Publish thruster commands
    std_msgs::Float32 left;
    left.data = static_cast<float>(cmdThrust[0]);
    std_msgs::Float32 right;
    right.data = static_cast<float>(cmdThrust[1]);
    m_leftPub.publish(left);
    m_rightPub.publish(right);

    ros::spinOnce();
    rate.sleep();
  }
}

double OtterController::calculateSurgeForce(double deltaTime, double u)
{
  static double integralTerm = 0.0;

  double u_d_dot = 0.0;
  double u_tilde = u - u_d;

  integralTerm += u_tilde * deltaTime;

  return mass_u * (u_d_dot - Kp_u * u_tilde - Ki_u * integralTerm) + damp_u * u;
}

double OtterController::calculateYawMoment(double deltaTime, double psi_slam, double r)
{
  // TODO: reference model

  ROS_INFO_STREAM("Psi: " << psi_slam);

  static double integralTerm = 0.0;

  double r_d_dot = 0.0;
  double r_tilde = 0.0; // r - r_d;
  double psi_tilde = psi_slam - psi_d;
  if (psi_tilde > M_PI) {
    psi_tilde -= 2 * M_PI;
  } else if (psi_tilde < -M_PI) {
    psi_tilde += 2 * M_PI;
  }

  // TODO: anti windup
  //integralTerm += psi_tilde * deltaTime;

  return mass_psi * (r_d_dot - Kd_psi * r_tilde - Kp_psi * psi_tilde - Ki_psi * integralTerm) - damp_psi * r;
}

Eigen::Vector2d OtterController::thrustAllocation(Eigen::Vector3d tau_d)
{
  // Initialize thruster configuration matrix pseudoinverse
  static bool initialized = false;
  static Eigen::MatrixXd pinv(3, 2);
  if (!initialized) {
    initialized = true;
    pinv = T.completeOrthogonalDecomposition().pseudoInverse();
  }

  // Calculate thruster output
  Eigen::Vector2d u = pinv * tau_d;

  // Ensure in interval [-1, 1]
  u[0] = std::min(std::max(u[0], -1.0), 1.0);
  u[1] = std::min(std::max(u[1], -1.0), 1.0);

  return u;
}

void OtterController::inputCallback(const usv_msgs::SpeedCourse& msg)
{
  u_d = msg.speed;
  psi_d = msg.course;
  ROS_INFO_STREAM("Psi_d: " << psi_d);
}

void OtterController::speedCallback(const geometry_msgs::Vector3Stamped& msg)
{
  // GPS in ENU => rotate speed vector pi/2
  double dir = std::atan2(msg.vector.y, msg.vector.x) + M_PI_2;
  ROS_INFO_STREAM("Speed dir: " << dir);
  if (std::abs(dir - psi) > M_PI_2 && std::abs(dir - psi) < 3 * M_PI_2) {
    u = -std::sqrt(std::pow(msg.vector.x, 2) + std::pow(msg.vector.y, 2));
  } else {
    u = std::sqrt(std::pow(msg.vector.x, 2) + std::pow(msg.vector.y, 2));
  }
}

void OtterController::imuCallback(const sensor_msgs::Imu& msg)
{
  psi = tf2::getYaw(msg.orientation);
  r = msg.angular_velocity.z;
}

double OtterController::getYaw()
{
  static tf2_ros::Buffer tfBuffer;
  static tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped tfStamped;
  try {
    tfStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0), ros::Duration(1.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  return tf2::getYaw(tfStamped.transform.rotation);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "OtterController");
  OtterController otterController;
  return 0;
}
