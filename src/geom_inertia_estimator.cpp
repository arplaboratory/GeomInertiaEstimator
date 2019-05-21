#include "geom_inertia_estimator.h"

////////////////////  MAIN FUNCTIONS  ////////////////////

void InertiaEstimator::onInit(const ros::NodeHandle &nh)
{
  // initialize often used variables as consts
  consts_.zero3 = mat3::Zero();
  consts_.eye3 = mat3::Identity();
  consts_.eye_NST = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity();
  consts_.rotate3 = Eigen::DiagonalMatrix<flt,3>(1,-1,-1);
  //consts_.rotate6 << consts_.rotate3, consts_.zero3, consts_.zero3, consts_.rotate3;
  consts_.e_z << 0.0f,0.0f,1.0f;
  consts_.g = 9.807f;

  init_vars_.P0 = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Zero();

  // read flag to see whether to use EKF or UKF
  double tempd;
  nh.param("useEKF", tempd, 1.0);
  useEKF_ = bool(tempd);

  // temp variables for faster reads
  Eigen::Matrix<double,30,1> temp30;
  Eigen::Matrix<double,6,1>   temp6;
  Eigen::Matrix<double,3,1>   temp3;
  Eigen::Matrix<double,2,1>   temp2;
  double                      temp;

  // read attitude of IMU
  nh.param("RImu/a/x",     temp6(0), 5.0);
  nh.param("RImu/a/y",     temp6(1), 5.0);
  nh.param("RImu/a/z",     temp6(2), 5.0);
  nh.param("RImu/Omega/x", temp6(3), 1.0);
  nh.param("RImu/Omega/y", temp6(4), 1.0);
  nh.param("RImu/Omega/z", temp6(5), 1.0);
  consts_.RImu = (temp6.template cast<flt>()).asDiagonal();

  // read process covariance
  std::vector<flt> Qx_temp(NUM_STATES_TANGENT);
  nh.param("Qx",Qx_temp, std::vector<flt>(NUM_STATES_TANGENT));
  consts_.Qx = Eigen::Map<Eigen::Matrix<flt,NUM_STATES_TANGENT,1> >(&Qx_temp[0]).asDiagonal();

  // read initial values
  // moment of inertia
  nh.param("x0/I/x", temp3(0), 1.0*1e-3);
  nh.param("x0/I/y", temp3(1), 1.0*1e-3);
  nh.param("x0/I/z", temp3(2), 1.0*1e-3);
  init_vars_.I = (temp3.cast<flt>()).asDiagonal();
  // mass
  nh.param("x0/m",      init_vars_.m, 0.5);
  // position center of mass
  nh.param("x0/r_BM/x", temp3(0), 0.0);
  nh.param("x0/r_BM/y", temp3(1), 0.0);
  nh.param("x0/r_BM/z", temp3(2), 0.0);
  init_vars_.r_BM = temp3.cast<flt>();
  // position pose sensor
  nh.param("x0/r_BP/x", temp3(0), 0.0);
  nh.param("x0/r_BP/y", temp3(1), 0.0);
  nh.param("x0/r_BP/z", temp3(2), 0.0);
  init_vars_.r_BP = temp3.cast<flt>();
  // position IMU
  nh.param("x0/r_BI/x", temp2(0), 0.0);
  nh.param("x0/r_BI/y", temp2(1), 0.0);
  init_vars_.r_BI = temp2.cast<flt>();

  // read initial state covariance
  // velocity
  nh.param("P0/v/x",       temp30(6), 1.0*1e-3);
  nh.param("P0/v/y",       temp30(7), 1.0*1e-3);
  nh.param("P0/v/z",       temp30(8), 1.0*1e-3);
  // mass
  nh.param("P0/m",         temp30(12), 0.5);
  // moment of inertia
  nh.param("P0/I/x",       temp30(13), 1.0*1e-6);
  nh.param("P0/I/y",       temp30(14), 1.0*1e-6);
  nh.param("P0/I/z",       temp30(15), 1.0*1e-6);
  // position center of mass
  nh.param("P0/r_BM/x",    temp30(16), 0.1);
  nh.param("P0/r_BM/y",    temp30(17), 0.1);
  nh.param("P0/r_BM/z",    temp30(18), 0.1);
  // position pose sensor
  nh.param("P0/r_BP/x",    temp30(19), 0.1);
  nh.param("P0/r_BP/y",    temp30(20), 0.1);
  nh.param("P0/r_BP/z",    temp30(21), 0.1);
  // position IMU sensor
  nh.param("P0/r_BI/x",    temp30(22), 0.1);
  nh.param("P0/r_BI/y",    temp30(23), 0.1);
  // IMU bias acceleration
  nh.param("P0/b_a/x",     temp30(24), 4.0);
  nh.param("P0/b_a/y",     temp30(25), 4.0);
  nh.param("P0/b_a/z",     temp30(26), 4.0);
  // IMU bias angular velocity
  nh.param("P0/b_Omega/x", temp30(27), 1.0);
  nh.param("P0/b_Omega/y", temp30(28), 1.0);
  nh.param("P0/b_Omega/z", temp30(29), 1.0);
  init_vars_.P0 = (temp30.cast<flt>()).asDiagonal();

  // read UKF tuning parameters
  nh.param("UKF/alpha", temp, 0.4);
  consts_.alpha = flt(temp);
  nh.param("UKF/beta", temp, 2.0);
  consts_.beta = flt(temp);
  nh.param("UKF/kappa", temp, 0.0);
  consts_.kappa = flt(temp);

  // read attitude of pose sensor and IMU
  std::vector<flt> R_temp,t_temp;
  nh.param("R_BP", R_temp, std::vector<flt>(9,1));
  consts_.R_BP = Eigen::Map<Eigen::Matrix<flt,3,3,Eigen::RowMajor> >(&R_temp[0]);
  consts_.R_BP_6D << consts_.R_BP, consts_.zero3, consts_.zero3, consts_.R_BP;
  consts_.q_RB = quat(consts_.R_BP).normalized();
  nh.param("R_BI", R_temp, std::vector<flt>(9,1));
  consts_.R_BI = Eigen::Map<Eigen::Matrix<flt,3,3,Eigen::RowMajor> >(&R_temp[0]);
  Eigen::Matrix<flt,6,6> temp66;
  temp66 << consts_.R_BI, consts_.zero3, consts_.zero3, consts_.R_BI;
  consts_.RImu = temp66*consts_.RImu*temp66.transpose();

  // calcuate matrices for force & moment calculation for the case where M & B coincide
  mat3 R;
  vec3 t;
  double k_f, k_M;
  int num_rotors;
  Eigen::Matrix<flt,3,20> P_f,P_M;
  for (num_rotors = 0; num_rotors < 20; num_rotors++)
  {
    // read thrust coeff, drag moment coeff, attitude and position of rotors
    nh.param((std::string("multirotor/rotor" + std::to_string(num_rotors+1) + "/kf")).c_str(), k_f, 5.0e-9);
    nh.param((std::string("multirotor/rotor" + std::to_string(num_rotors+1) + "/km")).c_str(), k_M, 4.0e-11);
    nh.param((std::string("multirotor/rotor" + std::to_string(num_rotors+1) + "/R")).c_str(), R_temp, std::vector<flt>(9,1));
    nh.param((std::string("multirotor/rotor" + std::to_string(num_rotors+1) + "/t")).c_str(), t_temp, std::vector<flt>(3,1));
    R = Eigen::Map<Eigen::Matrix<flt,3,3,Eigen::RowMajor> >(&R_temp[0]);
    t = Eigen::Map<vec3>(&t_temp[0]);

    std::vector<flt> Qx_temp(NUM_STATES_TANGENT);
    nh.param("Qx",Qx_temp, std::vector<flt>(NUM_STATES_TANGENT));
    consts_.Qx = Eigen::Map<Eigen::Matrix<flt,NUM_STATES_TANGENT,1> >(&Qx_temp[0]).asDiagonal();

    if (R.norm() == 3) // not defined rotor
      break;
    else
    {
      // TODO save individual kf/km for each rotor
      consts_.k_f = k_f;
      consts_.k_M = k_M;
      P_f.col(num_rotors) = consts_.k_f*R*consts_.e_z;
      P_M.col(num_rotors) = consts_.k_M*R*consts_.e_z + t.cross(consts_.k_f*R*consts_.e_z);
    }
  }
  consts_.P_f = P_f.block(0,0,3,num_rotors);
  consts_.P_M = P_M.block(0,0,3,num_rotors);
  consts_.num_rotors = num_rotors;

  // calculate the lambda and weights for unscented transform
  consts_.lambda = consts_.alpha*consts_.alpha*(NUM_STATES_TANGENT+consts_.kappa) - (NUM_STATES_TANGENT);
  consts_.Wm << consts_.lambda/(NUM_STATES_TANGENT + consts_.lambda), Eigen::Matrix<flt,2*(NUM_STATES_TANGENT),1>::Constant(2*(NUM_STATES_TANGENT),1,1.0/(2.0*((NUM_STATES_TANGENT) + consts_.lambda)));
  consts_.Wc << consts_.lambda/(NUM_STATES_TANGENT + consts_.lambda) + (1-consts_.alpha*consts_.alpha+consts_.beta), Eigen::Matrix<flt,2*(NUM_STATES_TANGENT),1>::Constant(2*(NUM_STATES_TANGENT),1,1.0/(2.0*(NUM_STATES_TANGENT + consts_.lambda)));
}

void InertiaEstimator::initializeFilter(const InitVars &init)
{
  // set state value, state covariance values and time stamp
  state_.X.r = init.r;
  state_.X.q = init.q;
  state_.X.Omega = init.Omega;

  state_.X.I = init.I;
  state_.X.m = init.m;
  state_.X.r_BM = init.r_BM;
  state_.X.r_BP = init.r_BP;
  state_.X.r_BI = init.r_BI;

  state_.X.b_a = vec3::Zero();
  state_.X.b_Omega = vec3::Zero();

  state_.t = init.t;

  state_.P = init.P0;

  init_vars_ = InitVars();
  initialized_ = true;

  publishEstimates(state_);

  print("filter initialized");
}

void InertiaEstimator::resetFilter()
{
  init_vars_ = InitVars();
  initialized_ = false;
  warn("reset",int(init_vars_.rpmInitialized || init_vars_.imuInitialized || init_vars_.poseInitialized));
}

void InertiaEstimator::predictEKF(StateWithCov &state, Input &input)
{
  // calculate time diff to predicted state
  flt dt = (input.t-state.t).toSec();
  if (dt > 1.0e-5)
  {
    angleA orientation = qLog(state.X.q)*2.0;
    // calculate state transition matrix F using matlab generated function
    calc_EKF_F_optimized(
          orientation(0),orientation(1),orientation(2),
          state.X.r(0),state.X.r(1),state.X.r(2),
          state.X.v(0),state.X.v(1),state.X.v(2),
          state.X.Omega(0),state.X.Omega(1),state.X.Omega(2),
          state.X.m,
          state.X.I(0,0),state.X.I(1,1),state.X.I(2,2),
          state.X.r_BM(0),state.X.r_BM(1),state.X.r_BM(2),
          state.X.r_BP(0),state.X.r_BP(1),state.X.r_BP(2),
          state.X.r_BI(0),state.X.r_BI(1),
          state.X.b_a(0),state.X.b_a(1),state.X.b_a(2),
          state.X.b_Omega(0),state.X.b_Omega(1),state.X.b_Omega(2),
          std::sqrt(input.rpm_sq(0)),std::sqrt(input.rpm_sq(1)),std::sqrt(input.rpm_sq(2)),std::sqrt(input.rpm_sq(3)),
          dt,
          0,
          0.08f, 0.071f, 0.0095f, 4.179e-9f, F_temp_);

    // use with doubles
    Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> F(Eigen::Matrix<double,NUM_STATES_TANGENT,NUM_STATES_TANGENT>(F_temp_).cast<flt>());
    // use with floats
    //Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> F(F_temp);

    // update time stamp
    state.t = input.t;
    // predicted mean state with non-linear model
    processModel(state.X,input,dt);
    // predicted state covariance with linearized model
    state.P = F*state.P*F.transpose() + consts_.Qx*dt;
  }
}

void InertiaEstimator::predictUKF(StateWithCov &state, Input &input)
{
  // calculate time diff to predicted state
  flt dt = (input.t - state.t).toSec();

  if (dt > 1.0e-5) {
    // create and propagate sigma points
    State sigmaPoints[NUM_SIGMAS];
    getSigmaPoints(sigmaPoints,state);
    for (int i = 0; i<NUM_SIGMAS; i++)
      processModel(sigmaPoints[i],input,dt);
  
    // calculate the mean    
    calculateMeanStateSigma(state.X, sigmaPoints);
  
    // calculate the covariance
    Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_SIGMAS> diff;
    for (int i=0; i<NUM_SIGMAS; i++) {
      diff.col(i) = sigmaPoints[i].boxminus(state.X);
    }
    state.P = diff*consts_.Wc.asDiagonal()*diff.transpose() + consts_.Qx*dt;

    // update time stamp and
    state.t = input.t;
  }
}

void InertiaEstimator::measUpdatePoseEKF(StateWithCov &state, MeasPose &meas)
{
  // since orientation measurement is linear, we use linear Kalman Filter update for orientation measurement
  // allocate matrices
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H.block(0,0,3,3) = mat3::Identity();
  // innovation
  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = qBoxMinus(meas.q,state.X.q);
  // innovation covariance
  Eigen::Matrix<flt,3,3> S_KF = meas.cov.block(0,0,3,3) + H*state.P*H.transpose();
  // optimal kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H.transpose() * S_KF.inverse();
  // updated state estimate and state covariance estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;
  state.X.boxplus(dX);
  state.P = (consts_.eye_NST-K_KF*H)*state.P*(consts_.eye_NST-K_KF*H).transpose() + K_KF*meas.cov.block(0,0,3,3)*K_KF.transpose();

  // non-linear update for position measurement
  // prep calculations for the calculation of linearized observation model (H)
  angleA orientation = qLog(state.X.q)*2.0;
  double H_temp[90];
  // calculate linearized observation model using Matlab generated function
  calc_EKF_H_odom_optimized_simple(
        orientation(0),orientation(1),orientation(2),
        state.X.r(0),state.X.r(1),state.X.r(2),
        state.X.v(0),state.X.v(1),state.X.v(2),
        state.X.Omega(0),state.X.Omega(1),state.X.Omega(2),
        state.X.m,
        state.X.I(0,0),state.X.I(1,1),state.X.I(2,2),
        state.X.r_BM(0),state.X.r_BM(1),state.X.r_BM(2),
        state.X.r_BP(0),state.X.r_BP(1),state.X.r_BP(2),
        state.X.r_BI(0),state.X.r_BI(1),
        state.X.b_a(0),state.X.b_a(1),state.X.b_a(2),
        state.X.b_Omega(0),state.X.b_Omega(1),state.X.b_Omega(2),
        0,0,0,0,
        0,
        0,
        0.08f, 0.071f, 0.0095f, 4.179e-9f, H_temp);

  // use with floats
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_EKF(Eigen::Matrix<double,3,NUM_STATES_TANGENT>(H_temp).cast<flt>());
  // use with doubles
  //Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_EKF(H_temp);

  // mean of expected measurement using non-linear measurement model
  vec3 zpred = measurementModelPosition(state.X);
  // innovation covariance
  mat3 S_EKF = H_EKF*state.P*H_EKF.transpose() + meas.cov.block(3,3,3,3);
  // kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_EKF = state.P*H_EKF.transpose()*S_EKF.inverse();
  // updated state estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_EKF*(meas.r - zpred);
  state.X.boxplus(del);
  // updated state estimate covariance (a posteriori)
  state.P = (Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity() - K_EKF*H_EKF) * state.P;
}

void InertiaEstimator::measUpdatePoseUKF(StateWithCov &state, MeasPose &meas)
{
  // since orientation measurement is linear, we use linear Kalman Filter update for orientation measurement
  // allocate matrices
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H.block(0,0,3,3) = mat3::Identity();
  // innovation
  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = qBoxMinus(meas.q,state.X.q);
  // innovation covariance
  Eigen::Matrix<flt,3,3> S_KF = meas.cov.block(0,0,3,3) + H*state.P*H.transpose();
  // optimal kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H.transpose() * S_KF.inverse();
  // updated state estimate and state covariance estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;
  state.X.boxplus(dX);
  state.P = (consts_.eye_NST-K_KF*H)*state.P*(consts_.eye_NST-K_KF*H).transpose() + K_KF*meas.cov.block(0,0,3,3)*K_KF.transpose();

  // Unscented Kalman Filter update step for position measurement
  // allocating sigma points
  const int L = 2*(NUM_STATES_TANGENT)+1;
  State sigmaPoints[L];
  Eigen::Matrix<flt,3,L> propagatedSigmaPointsMat;
  // calculate sigma points
  getSigmaPoints(sigmaPoints,state);
  // sigmapoint transformation through non-linear measurement model
  for (int i = 0; i<L; i++)
    propagatedSigmaPointsMat.col(i) = measurementModelPosition(sigmaPoints[i]);
  // mean of transformed sigma points
  vec3 zhat = propagatedSigmaPointsMat*consts_.Wm;
  // innovation
  Eigen::Matrix<flt,3,L> diffZ;
  diffZ = propagatedSigmaPointsMat - zhat.replicate(1,L);
  // calculate innovation covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,L> diffX;
  for (int i=0; i<L; i++) {
    diffX.col(i) = sigmaPoints[i].boxminus(state.X);
  }
  mat3 S_UKF  = diffZ * consts_.Wc.asDiagonal() * diffZ.transpose() + meas.cov.block(3,3,3,3);
  // kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> Pcross = diffX * consts_.Wc.asDiagonal() * diffZ.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_UKF = Pcross*S_UKF.inverse();
  // updated covariance estimate (a posteriori)
  state.P = state.P - K_UKF*S_UKF*K_UKF.transpose();
  // updated estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_UKF*(meas.r - zhat);
  state.X.boxplus(del);
}

void InertiaEstimator::measUpdateImuEKF(StateWithCov &state, Input &input, MeasImu &meas)
{
  // since angular velocity measurement is linear, we use linear Kalman Filter update for orientation measurement
  // allocate matrices
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_P = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H_P.block(0,9,3,3) = mat3::Identity();
  H_P.block(0,27,3,3) = mat3::Identity();
  // innovation
  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = meas.Omega - (state.X.Omega + state.X.b_Omega);
  // innovation covariance
  Eigen::Matrix<flt,3,3> S_KF = H_P*state.P*H_P.transpose() + consts_.RImu.block(3,3,3,3);
  // optimal kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H_P.transpose() * S_KF.inverse();
  // updated state estimate and state covariance estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;
  state.X.boxplus(dX);
  state.P = (consts_.eye_NST-K_KF*H_P)*state.P*(consts_.eye_NST-K_KF*H_P).transpose() + K_KF*consts_.RImu.block(3,3,3,3)*K_KF.transpose();

  // non-linear update for acceleration measurement
  // prep caluclaitons for the calculation of linearized observation model (H)
  angleA orientation = qLog(state.X.q)*2.0;
  flt dt = (input.t-state.t).toSec();
  double H_temp[90];
  // calculate linearized observation model using Matlab generated function
  calc_EKF_H_imu_optimized_simple(
        orientation(0),orientation(1),orientation(2),
        state.X.r(0),state.X.r(1),state.X.r(2),
        state.X.v(0),state.X.v(1),state.X.v(2),
        state.X.Omega(0),state.X.Omega(1),state.X.Omega(2),
        state.X.m,
        state.X.I(0,0),state.X.I(1,1),state.X.I(2,2),
        state.X.r_BM(0),state.X.r_BM(1),state.X.r_BM(2),
        state.X.r_BP(0),state.X.r_BP(1),state.X.r_BP(2),
        state.X.r_BI(0),state.X.r_BI(1),
        state.X.b_a(0),state.X.b_a(1),state.X.b_a(2),
        state.X.b_Omega(0),state.X.b_Omega(1),state.X.b_Omega(2),
        std::sqrt(input.rpm_sq(0)),std::sqrt(input.rpm_sq(1)),std::sqrt(input.rpm_sq(2)),std::sqrt(input.rpm_sq(3)),
        dt,
        0,
        0.08f, 0.071f, 0.0095f, 4.179e-9f, H_temp);

  // use with floats
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_EKF(Eigen::Matrix<double,3,NUM_STATES_TANGENT>(H_temp).cast<flt>());
  // use with doubles
  //Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_EKF(H_temp);

  // mean of expected measurement using non-linear measurement model
  vec3 zpred = measurementModelAcceleration(state.X,input);
  // innovation covariance
  mat3 S_EKF = H_EKF*state.P*H_EKF.transpose() + consts_.RImu.block(0,0,3,3);
  // kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_EKF = state.P*H_EKF.transpose()*(S_EKF.inverse());
  // updated state estimate (a posteriori)
  vec3 yhat = meas.a - zpred;
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_EKF*yhat;
  state.X.boxplus(del);
  // updated state estimate covariance (a posteriori)
  state.P = ( Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity() - K_EKF*H_EKF) * state.P;
}

void InertiaEstimator::measUpdateImuUKF(StateWithCov &state, Input &input, MeasImu &meas)
{
  // since angular velocity measurement is linear, we use linear Kalman Filter update for orientation measurement
  // allocate matrices
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_P = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H_P.block(0,9,3,3) = mat3::Identity();
  H_P.block(0,27,3,3) = mat3::Identity();
  // innovation
  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = meas.Omega - (state.X.Omega + state.X.b_Omega);
  // innovation covvariance
  Eigen::Matrix<flt,3,3> S_KF = consts_.RImu.block(3,3,3,3) + H_P*state.P*H_P.transpose();
  // optimal kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H_P.transpose() * S_KF.inverse();
  // updated state estimate and state covariance estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;
  state.X.boxplus(dX);
  state.P = (consts_.eye_NST-K_KF*H_P)*state.P*(consts_.eye_NST-K_KF*H_P).transpose() + K_KF*consts_.RImu.block(3,3,3,3)*K_KF.transpose();

  // Unscented Kalman Filter update step for acceleration measurement
  // allocating sigma points
  const int L = 2*(NUM_STATES_TANGENT)+1;
  State sigmaPoints[L];
  Eigen::Matrix<flt,3,L> propagatedSigmaPointsMat;
  // calculate sigma points
  getSigmaPoints(sigmaPoints,state);
  // sigmapoint transformation through non-linear measurement model
  for (int i = 0; i<L; i++)
    propagatedSigmaPointsMat.col(i) = measurementModelAcceleration(sigmaPoints[i],input);
  // mean of transformed sigma points
  vec3 zhat = propagatedSigmaPointsMat*consts_.Wm;
  // innovation
  Eigen::Matrix<flt,3,L> diffZ;
  diffZ = propagatedSigmaPointsMat - zhat.replicate(1,L);
  // calculate innovation covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,L> diffX;
  for (int i=0; i<L; i++) {
    diffX.col(i) = sigmaPoints[i].boxminus(state.X);
  }
  mat3 S_UKF  = diffZ * consts_.Wc.asDiagonal() * diffZ.transpose() + consts_.RImu.block(0,0,3,3);
  // kalman gain
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> Pcross = diffX * consts_.Wc.asDiagonal() * diffZ.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_UKF = Pcross*S_UKF.inverse();
  // updated covariance estimate (a posteriori)
  state.P = state.P - K_UKF*S_UKF*K_UKF.transpose();
  // updated estimate (a posteriori)
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_UKF*(meas.a - zhat);
  state.X.boxplus(del);
}


////////////////////  IMPLEMENTATION FUNCTIONS  ////////////////////

////////////////////  process and measurement models  ////////////////////

void InertiaEstimator::processModel(State &X, Input &U, flt &dt)
{
  // correct wrench calculation matrix for the case of the center of mass not being in the geometric center of rotors
  static Eigen::Matrix<flt,3,4> P_M_CoGcorrected;
  P_M_CoGcorrected = consts_.P_M + consts_.k_f*Eigen::Matrix<flt,3,1>(-X.r_BM(1),X.r_BM(0),0.0).replicate(1,consts_.num_rotors);
  // calculate delta: delta = xdot*dt
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> delta;
  delta <<  X.Omega*dt,
            X.v*dt,
            (1.0/X.m*qRotateVec(X.q,consts_.P_f*U.rpm_sq) - consts_.g*consts_.e_z)*dt,
            (X.I.inverse()*((P_M_CoGcorrected*U.rpm_sq)-X.Omega.cross(X.I*X.Omega)))*dt,
            0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
            0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f;
  // add delta: x_new = x_old + delta
  X.boxplus(delta);
}

Eigen::Matrix<flt,3,1> InertiaEstimator::measurementModelPosition(State &X)
{
  // measurement model position
  Eigen::Matrix<flt,3,1> meas = X.r + qRotateVec(X.q,X.r_BP-X.r_BM);
  return meas;
}

Eigen::Matrix<flt,3,1> InertiaEstimator::measurementModelAcceleration(State &X, Input &U)
{
  // vector from center of mass to IMU
  vec3 r_MI = vec3(X.r_BI.x(),X.r_BI.y(),0) - X.r_BM;
  // correct wrench calculation matrix for the case of the center of mass not being in the geometric center of rotors
  static Eigen::Matrix<flt,3,Eigen::Dynamic> P_M_CoGcorrected;
  P_M_CoGcorrected = consts_.P_M + consts_.k_f*Eigen::Matrix<flt,3,1>(-X.r_BM(1),X.r_BM(0),0.0).replicate(1,consts_.num_rotors);
  // applied force and moment based on rotor rpms
  vec3 F = consts_.P_f*     U.rpm_sq;
  vec3 M = P_M_CoGcorrected*U.rpm_sq;
  // measurement model acceleration
  vec3 Omega_dot = X.I.inverse()*(M-X.Omega.cross(X.I*X.Omega));
  vec3 a_meas = 1.0/X.m*F
                + Omega_dot.cross(r_MI)
                + X.Omega.cross(X.Omega.cross(r_MI))
                + X.b_a;
  return a_meas;
}

////////////////////  UKF functions  ////////////////////

void InertiaEstimator::getSigmaPoints(State *sigmaPoints, StateWithCov &state)
{
  // Cholesky factorization
  Eigen::LLT<Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>> factorization(state.P);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> sqrt = factorization.matrixL();
  flt c = std::sqrt(consts_.lambda + NUM_STATES_TANGENT);
  sqrt *= c;

  // calculate sigma points
  for (int i = 0; i <= 2*NUM_STATES_TANGENT; i++)
    sigmaPoints[i] = state.X;
  for (int i = 1; i <= NUM_STATES_TANGENT; i++){
    sigmaPoints[i].boxplus(sqrt.col(i-1));
    sigmaPoints[i+NUM_STATES_TANGENT].boxplus(-sqrt.col(i-1));
  }
}

void InertiaEstimator::calculateMeanStateSigma(State &mean, const State sigmaPoints[NUM_SIGMAS])
{
  // function to calculate the mean of sigma points
  // calculate mean on SO(3)
  quat qList[NUM_SIGMAS];
  for (int i = 0; i < NUM_SIGMAS; i++)
    qList[i] = sigmaPoints[i].q;
  mean.q = qMean(qList,consts_.Wm);
  // calulate mean on Euclidean Space variables
  mean.r       = consts_.Wm[0] * sigmaPoints[0].r;
  mean.v       = consts_.Wm[0] * sigmaPoints[0].v;
  mean.Omega   = consts_.Wm[0] * sigmaPoints[0].Omega;
  mean.m       = consts_.Wm[0] * sigmaPoints[0].m;
  mean.r_BM    = consts_.Wm[0] * sigmaPoints[0].r_BM;
  mean.r_BP    = consts_.Wm[0] * sigmaPoints[0].r_BP;
  mean.r_BI    = consts_.Wm[0] * sigmaPoints[0].r_BI;
  mean.b_a     = consts_.Wm[0] * sigmaPoints[0].b_a;
  mean.b_Omega = consts_.Wm[0] * sigmaPoints[0].b_Omega;
  for (int i = 1; i < NUM_SIGMAS; i++)
  {
    mean.r       += consts_.Wm[i] * sigmaPoints[i].r;
    mean.v       += consts_.Wm[i] * sigmaPoints[i].v;
    mean.Omega   += consts_.Wm[i] * sigmaPoints[i].Omega;
    mean.m       += consts_.Wm[i] * sigmaPoints[i].m;
    mean.r_BM    += consts_.Wm[i] * sigmaPoints[i].r_BM;
    mean.r_BP    += consts_.Wm[i] * sigmaPoints[i].r_BP;
    mean.r_BI    += consts_.Wm[i] * sigmaPoints[i].r_BI;
    mean.b_a     += consts_.Wm[i] * sigmaPoints[i].b_a;
    mean.b_Omega += consts_.Wm[i] * sigmaPoints[i].b_Omega;
  }
}

////////////////////  message callback functions  ////////////////////

void InertiaEstimator::rpm_callback(const geom_inertia_estimator::MotorRPM::ConstPtr &msg)
{
  // read msg
  rpmMsg2input(input_,msg);
  if (initialized_)
  {
    // predict state with either EKF or UKF
    if (useEKF_)
      predictEKF(state_, input_);
    else
      predictUKF(state_, input_);
    // publish updated estimates
    publishEstimates(state_);
  }
  else
  {
    // if not initialized, save msg, set flag that pose initialized
    if (inputList_.size() >= 30)
      inputList_.pop();
    init_vars_.rpmInitialized = true;
    init_vars_.t = msg->header.stamp;
    // check whether ready to initialize or not
    init_vars_.readyToInitialize = init_vars_.rpmInitialized &&
                                   init_vars_.imuInitialized &&
                                   init_vars_.poseInitialized;
    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

void InertiaEstimator::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  // read msg
  MeasPose meas_pose = poseMsg2measPose(msg);
  if (initialized_)
  {
    // update state with either EKF or UKF
    if (useEKF_)
      measUpdatePoseEKF(state_, meas_pose);
    else
      measUpdatePoseUKF(state_, meas_pose);
    // publish updated estimates
    publishEstimates(state_);
  }
  else
  {
    // if not initialized, save msg, set flag that pose initialized
    init_vars_.r   = meas_pose.r;
    init_vars_.q   = meas_pose.q;
    init_vars_.P0.block(0,0,3,3) = meas_pose.cov.block(0,0,3,3);
    init_vars_.P0.block(3,3,3,3) = meas_pose.cov.block(3,3,3,3);
    init_vars_.poseInitialized = true;
    // check whether ready to initialize or not
    init_vars_.readyToInitialize = init_vars_.rpmInitialized &&
                                   init_vars_.imuInitialized &&
                                   init_vars_.poseInitialized;
    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

void InertiaEstimator::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  // read message
  MeasImu meas = imuMsg2measImu(msg);
  if (initialized_)
  {
      // update state with either EKF or UKF
      if (useEKF_)
        measUpdateImuEKF(state_,input_,meas);
      else
        measUpdateImuUKF(state_,input_,meas);
      // publish updated estimates
      publishEstimates(state_);
  }
  else
  {
    // if not initialized, save msg, set flag that imu initialized
    meas = imuMsg2measImu(msg);
    init_vars_.Omega = meas.Omega;
    init_vars_.P0.block(9,9,3,3) = consts_.RImu.block(3,3,3,3);
    init_vars_.imuInitialized = true;
    // check whether ready to initialize
    init_vars_.readyToInitialize = init_vars_.rpmInitialized &&
                                   init_vars_.imuInitialized &&
                                   init_vars_.poseInitialized;
    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

////////////////////  message conversion functions  ////////////////////

void InertiaEstimator::rpmMsg2input(Input &returnVar, const geom_inertia_estimator::MotorRPM::ConstPtr &msg)
{
  // TODO adapt to arbitrary number of rotors
  // square rpms so we only need to multiply by the rotor thrust coeff
  returnVar.rpm_sq << msg->rpm[0]*msg->rpm[0],
                      msg->rpm[1]*msg->rpm[1],
                      msg->rpm[2]*msg->rpm[2],
                      msg->rpm[3]*msg->rpm[3];
  returnVar.t = msg->header.stamp;
}

MeasPose InertiaEstimator:: poseMsg2measPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  // TODO allow any rotation
  MeasPose meas_pose;
  // read position
  meas_pose.r = consts_.R_BP*vec3(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
  // read attitude
  meas_pose.q = consts_.q_RB*quat(msg->pose.pose.orientation.w,
                                  msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y,
                                  msg->pose.pose.orientation.z).normalized()*consts_.q_RB.conjugate();

  // rotate s.t. z is up instead of down
  mat6 covPose;
  getCovInMsg(msg->pose.covariance,covPose);
  covPose = consts_.R_BP_6D*covPose*consts_.R_BP_6D.transpose();

  meas_pose.cov = covPose;

  return meas_pose;
}

MeasImu InertiaEstimator::imuMsg2measImu(const sensor_msgs::Imu::ConstPtr &msg)
{
  MeasImu meas_imu;

  meas_imu.a     = consts_.R_BI*vec3(msg->linear_acceleration.x,
                                     msg->linear_acceleration.y,
                                     msg->linear_acceleration.z);

  meas_imu.Omega = consts_.R_BI*vec3(msg->angular_velocity.x,
                                     msg->angular_velocity.y,
                                     msg->angular_velocity.z);

  return meas_imu;
}

void InertiaEstimator::publishEstimates(const StateWithCov &estimate)
{
  // function to publish estimates and covariances
  static geom_inertia_estimator::ParameterEstimates msg;
  // set stamp
  msg.header.stamp = estimate.t;
  // set trajectory states
  msg.q = quat2msg(estimate.X.q);
  msg.r = vec2msg(estimate.X.r);
  msg.v = vec2msg(estimate.X.v);
  msg.Omega = vec2msg(estimate.X.Omega);
  // set inertia states
  msg.m    = estimate.X.m;
  msg.I    = vec2msg(estimate.X.I.diagonal());
  msg.r_BM = vec2msg(estimate.X.r_BM);
  // set geometric states
  msg.r_BP    = vec2msg(estimate.X.r_BP);
  msg.r_BI.x = estimate.X.r_BI(0);
  msg.r_BI.y = estimate.X.r_BI(1);
  // set biases states
  msg.b_a     = vec2msg(estimate.X.b_a);
  msg.b_Omega = vec2msg(estimate.X.b_Omega);
  // set covariance matrix
  for (int row = 0; row < NUM_STATES_TANGENT; row++)
    msg.covariance[row] = estimate.P(row,row);
  //publish
  pub_estimates_.publish(msg);
}

////////////////////  state addition/subraction functions  ////////////////////

void State::boxplus(const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta)
{
  // adding a delta (element of the tangent space of the state space) to a state (element of the state space)
  // "addition" on SO(3)
  this->q       = qBoxPlus(this->q,delta.segment(0,3));
  // addition in the Euclidean Space
  this->r       += delta.segment(3,3);
  this->v       += delta.segment(6,3);
  this->Omega   += delta.segment(9,3);
  this->m       += delta(12);
  this->I       += delta.segment(13,3).asDiagonal();
  this->r_BM    += delta.segment(16,3);
  this->r_BP    += delta.segment(19,3);
  this->r_BI    += delta.segment(22,2);
  this->b_a     += delta.segment(24,3);
  this->b_Omega += delta.segment(27,3);
}

Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> State::boxminus(const State &other) const
{
  // calculating the delta (lying on the tangent space of the state space) of two state
  Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> delta;
  // delta on SO(3)
  delta.segment(0,3)  = qBoxMinus(this->q,other.q);
  // delta on the Euclidean Space
  delta.segment(3,3)  = this->r - other.r;
  delta.segment(6,3)  = this->v - other.v;
  delta.segment(9,3)  = this->Omega - other.Omega;
  delta(12)           = this->m - other.m;
  delta.segment(13,3) = this->I.diagonal() - other.I.diagonal();
  delta.segment(16,3) = this->r_BM - other.r_BM;
  delta.segment(19,3) = this->r_BP - other.r_BP;
  delta.segment(22,2) =(this->r_BI - other.r_BI).segment(0,2);
  delta.segment(24,3) = this->b_a - other.b_a;
  delta.segment(27,3) = this->b_Omega - other.b_Omega;
  return delta;
}

