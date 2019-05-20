#ifndef GEOMINERTIAESTIMATOR_H
#define GEOMINERTIAESTIMATOR_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geom_inertia_estimator/MotorRPM.h>
#include <geom_inertia_estimator/ParameterEstimates.h>
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

// define state & measurement sizes
#define NUM_STATES         int(31)
#define NUM_STATES_TANGENT int(30)
#define NUM_MEAS_POSE      int(7)
#define NUM_MEAS_IMU       int(6)
#define NUM_SIGMAS         int(2*(NUM_STATES_TANGENT)+1)
#define RT_PI  3.14159265358979323846
#define RT_PIF 3.1415927F

// structs/classes to cleanly save sates, inputs, measurements, and initialization values
struct InitVars
{
  // variables needed for filter initialization
  // flags
  bool readyToInitialize = false;
  bool poseInitialized = false;
  bool imuInitialized = false;
  bool rpmInitialized = false;
  // time
  ros::Time t;
  // pose
  vec3 r;
  quat q;
  // imu
  vec3 Omega;
  // biases
  vec3 b_a;
  vec3 b_Omega;
  // from param file - user set
  double m;
  mat3 I;
  vec3 r_BM;
  vec3 r_BP;
  vec2 r_BI;
  // uncertainty covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> P0;
};

class State
{
public:
  // state variables
  // trajectory
  vec3 r;
  vec3 v;
  quat q;
  vec3 Omega;
  // inertia
  double m;
  mat3 I;
  vec3 r_BM;
  // geometric
  vec3 r_BP;
  vec2 r_BI;
  // biases
  vec3 b_a;
  vec3 b_Omega;

  // state manifold operations to "add" & "subtract" states
  Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> boxminus(const State& other) const;
  void boxplus(const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta);
};

struct StateWithCov
{
  // complete state with covariance and timestamp
  State X;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> P;
  ros::Time t;
};

struct Input
{
  // inputs for prediction step
  vec4 rpm_sq;
  ros::Time t;
};

struct MeasPose
{
  // measurements for pose measurement update
  vec3 r;
  quat q;
  mat6 cov;
};

struct MeasImu
{
  // measurements for IMU measurement update
  vec3 a;
  vec3 Omega;
};

struct Consts
{
  // often used constants
  // covariance IMU
  mat6 RImu;

  // process model covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> Qx;

  // rotor thrust and drag moment coefficients
  flt k_f,k_M;
  // matrices for wrench calculation from rotor rpms
  Eigen::Matrix<flt,3,4> P_f,P_M;
  int num_rotors;

  // UKF tuning parameters
  flt alpha, kappa, beta;
  flt lambda;
  // UKF weight matrices
  Eigen::Matrix<flt,2*NUM_STATES_TANGENT+1,1> Wm,Wc;

  // generally often used constants
  vec3 e_z;
  flt g;
  // matrices to speed up calculation
  mat3 rotate3;
  mat6 rotate6;
  mat3 zero3;
  mat3 eye3;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> eye_NST;
};


// estimator class
class InertiaEstimator
{
public:
  // node initialization function
  void onInit(const ros::NodeHandle &nh);
  void resetFilter();

  // subscribers and publisher
  ros::Subscriber sub_imu_,
                  sub_odom_,
                  sub_rpm_;
  ros::Publisher pub_estimates_;
  void rpm_callback (const geom_inertia_estimator::MotorRPM::ConstPtr &msg);
  //void rpm_callback (const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void pose_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void imu_callback (const sensor_msgs::Imu::ConstPtr &msg);

private:
  // member variables
  Consts consts_;
  InitVars init_vars_;
  StateWithCov state_;
  Input input_;
  std::queue<Input> inputList_;
  std::queue<MeasImu> measImuList_;
  double F_temp_[NUM_STATES_TANGENT*NUM_STATES_TANGENT];
  Eigen::Matrix<flt,NUM_STATES,NUM_SIGMAS> propagatedSigmaPointsMat_;

  // status flags
  bool initialized_ = false;
  bool useEKF_ = true;

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
  Eigen::Matrix<flt,3,1> measurementModelPosition(State& X);
  Eigen::Matrix<flt,3,1> measurementModelAcceleration(State& X, Input &U);

  // UKF functions
  void getSigmaPoints(State* sigmaPoints, StateWithCov &state);
  void calculateMeanStateSigma(State &mean, const State sigmaPoints[NUM_SIGMAS]);

  // subscirber functions: messge conversion functions
  void rpmMsg2input(Input &returnVar, const geom_inertia_estimator::MotorRPM::ConstPtr &msg);
  MeasPose poseMsg2measPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  MeasImu imuMsg2measImu(const sensor_msgs::Imu::ConstPtr &msg);

  // publisher function
  void publishEstimates(const StateWithCov &estimate);
};

#endif // INERTIAESTIMATOR_H
