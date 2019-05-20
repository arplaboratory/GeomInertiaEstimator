#include <ros/ros.h>
#include "geom_inertia_estimator.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "geom_inertia_estimator");
  ros::NodeHandle nh("~");
  //getchar();

  InertiaEstimator estimator;
  estimator.onInit(nh);
  estimator.sub_rpm_  = nh.subscribe("rpm",  10,&InertiaEstimator::rpm_callback,  &estimator);
  estimator.sub_odom_ = nh.subscribe("pose", 10,&InertiaEstimator::pose_callback, &estimator);
  estimator.sub_imu_  = nh.subscribe("imu",  10,&InertiaEstimator::imu_callback,  &estimator);

  estimator.pub_estimates_ = nh.advertise<geom_inertia_estimator::ParameterEstimates>("param_estimates", 10);

  ros::spin();

  return 0;
}
