#include <ros/ros.h>
#include "inertia_estimator.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "inertia_estimator");
  ros::NodeHandle nh("~");
  //getchar();

  InertiaEstimator estimator;
  estimator.onInit(nh);
  estimator.sub_rpm_  = nh.subscribe("meas_rpm",    10,&InertiaEstimator::rpm_callback,&estimator);
  estimator.sub_odom_ = nh.subscribe("odom",        10,&InertiaEstimator::pose_callback,&estimator);
  estimator.sub_imu_  = nh.subscribe("imu",         10,&InertiaEstimator::imu_callback,&estimator);

  estimator.pub_estimates_ = nh.advertise<inertia_estimator::ParameterEstimates>("param_estimates", 10);

  ros::spin();

  return 0;
}
