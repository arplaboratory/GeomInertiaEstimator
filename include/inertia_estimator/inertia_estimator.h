#ifndef INERTIAESTIMATOR_H
#define INERTIAESTIMATOR_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <inertia_estimator/MotorRPM.h>
#include <inertia_estimator/ParameterEstimates.h>
#include <Eigen/Cholesky>
#include <strstream>
#include <string.h>
#include <vector>
#include "v_tools.h"
#include <math.h>
#include <cmath>
#include <string.h>
#include <queue>
#include <time.h>

//includes for Matlab generated functions
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "calc_EKF_F_optimized/calc_EKF_F_optimized.h"
#include "calc_EKF_F_optimized/rt_defines.h"
#include "calc_EKF_F_optimized/rt_nonfinite.h"
#include "calc_EKF_F_optimized/rtwtypes.h"
#include "calc_EKF_F_optimized/calc_EKF_F_optimized_types.h"
#include "calc_EKF_H_imu_optimized_simple/calc_EKF_H_imu_optimized_simple.h"
#include "calc_EKF_H_odom_optimized_simple/calc_EKF_H_odom_optimized_simple.h"


using namespace std;

// define state & measurement sizes & prints
#define NUM_STATES         int(31)
#define NUM_STATES_TANGENT int(30)
#define NUM_MEAS_POSE      int(7)
#define NUM_MEAS_IMU       int(6)
#define NUM_SIGMAS         int(2*(NUM_STATES_TANGENT)+1)
#define RT_PI  3.14159265358979323846
#define RT_PIF 3.1415927F

// structs to cleanly save sates, inputs and measurements
struct InitVars
{
  bool readyToInitialize = false;
  bool poseInitialized = false;
  bool imuInitialized = false;
  bool rpmInitialized = false;

  // time
  ros::Time t;

  // odom
  vec3 r;
  quat q;

  // imu
  vec3 Omega;

  // from param file
  double m;
  mat3 I;
  vec3 r_BM;
  vec3 r_BP;
  vec2 r_BI;

  vec3 b_a;
  vec3 b_Omega;

  // uncertainty covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> P0;
};

class State
{
public:  
  vec3 r;
  vec3 v;
  quat q;
  vec3 Omega;

  double m;
  mat3 I;
  vec3 r_BM;
  vec3 r_BP;
  vec2 r_BI;
  vec3 b_a;
  vec3 b_Omega;

  State boxplus(const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta);
  Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> boxminus(const State& other) const;
};

struct StateWithCov
{
  State X;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> P;
  ros::Time t;
};

struct Input
{
  vec4 rpm_sq;
  ros::Time t;
};

struct MeasPose
{
  vec3 r;
  quat q;
  mat6 cov;
};

struct MeasImu
{
  vec3 a;
  vec3 Omega;
};

struct Consts
{
  mat6 RImu;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> Qx;
  Eigen::Matrix<flt,NUM_MEAS_POSE,NUM_STATES> H_cov_poseUpdate;

  // matrices to calculate force and moment from motor speeds
  flt k_f,k_M;
  Eigen::Matrix<flt,3,4> P_f,P_M;
  int num_rotors;

  // matrices to make calculations faster
  mat3 rotate3;
  mat6 rotate6;
  mat3 zero3;
  mat3 eye3;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> eye_NST;

  // UKF tuning parameters
  flt alpha, kappa, beta;
  flt lambda;
  Eigen::Matrix<flt,2*NUM_STATES_TANGENT+1,1> Wm,Wc;

  // generally often used constants
  vec3 e_z;
  flt g;
};


// estimator class
class InertiaEstimator
{
public:
  void onInit(const ros::NodeHandle &nh);
  void reset();

  // subscrieb to msgs
  ros::Subscriber sub_imu_,
                  sub_odom_,
                  sub_rpm_;
  ros::Publisher pub_estimates_;
//  void rpm_callback (const quadrotor_msgs::MotorRPM::ConstPtr &msg);
  void rpm_callback (const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void pose_callback (const nav_msgs::Odometry::ConstPtr &msg);
  void imu_callback (const sensor_msgs::Imu::ConstPtr &msg);

private:
  // member variables
  Consts consts_;
  InitVars init_vars_;
  StateWithCov state_;
  Input input_;
  std::queue<Input> inputList_;
  std::queue<MeasImu> measImuList_;

  // status
  bool initialized_ = false;
  bool useEKF_ = true;
  int IMU_skip_counter_ = 0;
  int RPM_skip_counter_ = 0;

  // timing functions
  clock_t ttt;
  double totTimePred, totTimeOdom, totTimeImu = 0.0;
  double counterPred, counterOdom, counterImu = 0.0;

  // estimator member function to initialize, predict and update
  void initializeFilter(const InitVars& init);
  void predictEKF(StateWithCov& oldState,Input& input);
  void predictUKF(StateWithCov &oldState, Input &input);
  void measUpdatePoseEKF(StateWithCov &oldState, MeasPose &meas);
  void measUpdatePoseUKF(StateWithCov &state, MeasPose &meas);
  void measUpdateImuEKF(StateWithCov& oldState, Input& input, MeasImu& meas);
  void measUpdateImuUKF(StateWithCov& state, Input& input, MeasImu& meas);

  // models
  void processModel(State& X, Input& U, flt &dt);
  Eigen::Matrix<flt,3,1> measurementModelPose(State& X);
  Eigen::Matrix<flt,3,1> measurementModelImu(State& X, Input &U);

  // UKF functions
  void getSigmaPoints(State* sigmaPoints, StateWithCov &state);
  void calculateMeanStateSigma(State &mean, const State sigmaPoints[NUM_SIGMAS]);
  // Input rpmMsg2input(const quadrotor_msgs::MotorRPM::ConstPtr &msg);
  void rpmMsg2input(Input &returnVar, const geometry_msgs::WrenchStamped::ConstPtr &msg);
  MeasPose poseMsg2measPose(const nav_msgs::Odometry::ConstPtr &msg);
  MeasImu imuMsg2measImu(const sensor_msgs::Imu::ConstPtr &msg);
  void publishEstimates(const StateWithCov &estimate);

  Eigen::Matrix<flt,NUM_STATES,1> state2stateVec(const State &X);
  State stateVec2state(Eigen::Matrix<flt,NUM_STATES,1> const &XV);

  double F_temp_[900];
  Eigen::Matrix<flt,NUM_STATES,NUM_SIGMAS> propagatedSigmaPointsMat_;
  void boxplus(State &X, const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta);
};

#endif // INERTIAESTIMATOR_H
