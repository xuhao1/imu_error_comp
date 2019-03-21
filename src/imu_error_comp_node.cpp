#include "imu_error_comp.h"

int main(int argc,char** argv)
{
  ros::init (argc, argv, "ImuErrorComp");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  ImuErrorComp imu_error_comp(nh,nh_private);
  ros::spin();
  
  return 0;
}