#include "imu_error_comp.h"
inline Eigen::Vector3d lowpass_filter(Eigen::Vector3d input, double fc, Eigen::Vector3d outputlast, double dt) {
	double RC = 1.0 / (fc *2 * M_PI);
	double alpha = dt / (RC + dt);
	return outputlast + (alpha* (input - outputlast));
}

ImuErrorComp::ImuErrorComp(ros::NodeHandle nh,ros::NodeHandle nh_private):
    nh_(nh),
    nh_private_(nh_private)
{
    ROS_INFO("Starting Imu Compensting...");

    // get param
    if(!nh_private_.getParam("use_gyro_intrinsic_params",use_gyro_intrinsic_params_))
    {
        use_gyro_intrinsic_params_ = false;
    }
        
    if(!nh_private_.getParam("use_acc_intrinsic_params",use_acc_intrinsic_params_))
    {
        use_acc_intrinsic_params_ = false;
    }

    if(!nh_private_.getParam("imu_subscribe_topic",imu_subscribe_topic_))
    {
        imu_subscribe_topic_ = "/imu/data_raw";
    }

    if (!nh_private_.getParam("imu_publish_topic", imu_publish_topic_))
    {
        imu_publish_topic_ = "/imu/data";
    }

    if (!nh_private_.getParam("accnorm_publish_topic", accnorm_publish_topic_))
    {
        accnorm_publish_topic_ = "/imu/accnorm";
    }
        
    if(!nh_private_.getParam("debug",debug_))
    {
        debug_ = false;
    }

    if (!nh_private_.getParam("pub_acc_norm", pub_acc_norm_))
    {
        pub_acc_norm_ = false;
    }

    if (!nh_private_.getParam("acc_norm_topic", debug_))
    {
        debug_ = false;
    }

    std::vector<double> mat_param;

    // get acc_Bias_ param
    mat_param.clear();
    acc_Bias_.setZero();
    if(nh_private_.getParam("acc_Bias",mat_param) && mat_param.size() == 3)
    {
        acc_Bias_ << mat_param[0],mat_param[1],mat_param[2];
    }
    else
    {
        ROS_INFO("can not get acc_Bias");
    }

    // get acc_Ea_ param
    mat_param.clear();
    acc_Ea_.setIdentity();
    if(nh_private_.getParam("acc_Ea",mat_param) && mat_param.size() == 9)
    {
        acc_Ea_ << 
        mat_param[0],mat_param[1],mat_param[2],
        mat_param[3],mat_param[4],mat_param[5],
        mat_param[6],mat_param[7],mat_param[8];
    }
    else
    {
        ROS_INFO("can not get acc_Ea");
    }

    // get acc_Ka_ param
    mat_param.clear();
    acc_Ka_.setZero();
    if(nh_private_.getParam("acc_Ka",mat_param) && mat_param.size() == 3)
    {
        acc_Ka_ << 
        mat_param[0],0,0,
        0,mat_param[1],0,
        0,0,mat_param[2];
    }
    else
    {
        ROS_INFO("can not get acc_Ka");
    }

    // get gyro_Bias_ param
    mat_param.clear();
    gyro_Bias_.setZero();
    if(nh_private_.getParam("gyro_Bias",mat_param) && mat_param.size() == 3)
    {
        gyro_Bias_ << mat_param[0],mat_param[1],mat_param[2];
    }
    else
    {
        ROS_INFO("can not get gyro_Bias");
    }

    // get gyro_Eg_ param
    mat_param.clear();
    gyro_Eg_.setIdentity();
    if(nh_private_.getParam("gyro_Eg",mat_param) && mat_param.size() == 9)
    {
        gyro_Eg_ << 
        mat_param[0],mat_param[1],mat_param[2],
        mat_param[3],mat_param[4],mat_param[5],
        mat_param[6],mat_param[7],mat_param[8];
    }
    else
    {
        ROS_INFO("can not get gyro_Eg");
    }

    // get gyro_Kg_ param
    mat_param.clear();
    gyro_Kg_.setZero();
    if(nh_private_.getParam("gyro_Kg",mat_param) && mat_param.size() == 3)
    {
        gyro_Kg_ << 
        mat_param[0],0,0,
        0,mat_param[1],0,
        0,0,mat_param[2];
    }
    else
    {
        ROS_INFO("can not get gyro_Kg");
    }    

    //imu_raw_sub_ = nh_.subscribe(imu_subscribe_topic_.c_str(),10,ImuErrorComp::imuCallback);
    imu_subscriber_.reset(new ImuSubscriber(nh_, imu_subscribe_topic_.c_str(), 10));
    imu_subscriber_->registerCallback(&ImuErrorComp::imuCallback, this);
    imu_comp_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_publish_topic_.c_str(),10);
    acc_norm_pub_ = nh_.advertise<geometry_msgs::Vector3>(accnorm_publish_topic_.c_str(), 10);
    acc_norm_pub_1 = nh_.advertise<geometry_msgs::Vector3>((accnorm_publish_topic_+"1").c_str(), 10);

    if(debug_)
    {
        std::cout << "imu_subscribe_topic_:" << imu_subscribe_topic_ << std::endl;
        std::cout << "imu_publish_topic_:" << imu_publish_topic_ << std::endl;
        std::cout << "use_gyro_intrinsic_params_:" << use_gyro_intrinsic_params_ << std::endl;
        std::cout << "use_acc_intrinsic_params_:" << use_acc_intrinsic_params_ << std::endl;
        std::cout << "debug_:" << debug_ << std::endl;

        std::cout <<  "acc_Bias_:" << "\n" << acc_Bias_ << std::endl;
        std::cout <<  "acc_Ea_:" << "\n" << acc_Ea_ << std::endl;
        std::cout <<  "acc_Ka_:" << "\n" << acc_Ka_ << std::endl;
        std::cout <<  "gyro_Bias_:" << "\n" << gyro_Bias_ << std::endl;
        std::cout <<  "gyro_Eg_:" << "\n" << gyro_Eg_ << std::endl;
        std::cout <<  "gyro_Kg_:" << "\n" << gyro_Kg_ << std::endl;
    }
}

ImuErrorComp::~ImuErrorComp()
{
}

void ImuErrorComp::imuCallback(const sensor_msgs::Imu::ConstPtr &imu)
{
    // get imu data
    // lowpass_filter(Eigen::Vector3d input, double fc, Eigen::Vector3d outputlast, double dt)
    gyro_data =  lowpass_filter( Eigen::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z), 
        40, gyro_data, 0.005);
    acc_data = lowpass_filter( Eigen::Vector3d(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z),
        40, acc_data, 0.005);

    if(pub_acc_norm_)
    {
        geometry_msgs::Vector3 imu_norm1;
        imu_norm1.x = sqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1] + acc_data[2] * acc_data[2]);
        imu_norm1.y = 0;
        imu_norm1.z = 0;
        acc_norm_pub_1.publish(imu_norm1);
    }


    if(use_acc_intrinsic_params_)
    {
		acc_data = acc_Ea_ * acc_Ka_ * (acc_data - acc_Bias_);
	}
	
	if(use_gyro_intrinsic_params_)
	{
		gyro_data = gyro_Eg_ * gyro_Kg_ * (gyro_data - gyro_Bias_);
	}

    // publish imu data
    sensor_msgs::Imu imu_pub = *imu;
    imu_pub.linear_acceleration.x = acc_data[0];
    imu_pub.linear_acceleration.y = acc_data[1];
    imu_pub.linear_acceleration.z = acc_data[2];

    imu_pub.angular_velocity.x = gyro_data[0];
    imu_pub.angular_velocity.y = gyro_data[1];
    imu_pub.angular_velocity.z = gyro_data[2];
    imu_comp_pub_.publish(imu_pub);

    if(pub_acc_norm_)
    {
        geometry_msgs::Vector3 imu_norm;
        imu_norm.x = sqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1] + acc_data[2] * acc_data[2]);
        imu_norm.y = 0;
        imu_norm.z = 0;
        acc_norm_pub_.publish(imu_norm);
    }
}