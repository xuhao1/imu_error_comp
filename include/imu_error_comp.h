#ifndef IMU_ERROR_COMP_H
#define IMU_ERROR_COMP_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <string>
#include <tf/tf.h>
#include <Eigen/Core>
#include <message_filters/subscriber.h>
#include <cmath>

class ImuErrorComp
{
  typedef message_filters::Subscriber<sensor_msgs::Imu> ImuSubscriber;
public:
   ImuErrorComp(ros::NodeHandle nh,ros::NodeHandle nh_private);
  ~ImuErrorComp();
private:
  //ros related 
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  boost::shared_ptr<ImuSubscriber> imu_subscriber_;
  ros::Publisher imu_comp_pub_;
  ros::Publisher acc_norm_pub_;
  ros::Publisher acc_norm_pub_1;

  //imu related
  bool use_gyro_intrinsic_params_;
  bool use_acc_intrinsic_params_;
  bool debug_;
  bool pub_acc_norm_;

  //topic name
  std::string imu_subscribe_topic_;
  std::string imu_publish_topic_;
  std::string accnorm_publish_topic_;

  //imu intrinsic parameters
  Eigen::Matrix<double,3,1>  acc_Bias_;
  Eigen::Matrix<double,3,3>  acc_Ea_;
  Eigen::Matrix<double,3,3>  acc_Ka_;

  Eigen::Matrix<double,3,1>  gyro_Bias_;
  Eigen::Matrix<double,3,3>  gyro_Eg_;
  Eigen::Matrix<double,3,3>  gyro_Kg_;
  
  Eigen::Matrix<double,3,1> gyro_data = Eigen::Vector3d(0, 0, 0);
  Eigen::Matrix<double,3,1> acc_data = Eigen::Vector3d(0, 0, 0);
  //imu call back
  void imuCallback(const sensor_msgs::Imu::ConstPtr &imu);
};
  
#endif