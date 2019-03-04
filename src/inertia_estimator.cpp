#include "inertia_estimator.h"

void InertiaEstimator::onInit(const ros::NodeHandle &nh)
{
  totTimePred = 0.0;
  totTimeOdom = 0.0;
  totTimeImu = 0.0;
  counterPred = 0.0;
  counterOdom = 0.0;
  counterImu = 0.0;

  consts_.zero3 = mat3::Zero();
  consts_.eye3 = mat3::Identity();
  consts_.eye_NST = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity();
  consts_.rotate3 = Eigen::DiagonalMatrix<flt,3>(1,-1,-1);
  consts_.rotate6 << consts_.rotate3, consts_.zero3, consts_.zero3, consts_.rotate3;
  consts_.e_z << 0.0f,0.0f,1.0f;
  consts_.g = 9.807f;

  init_vars_.P0 = Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Zero();

  double tempd;
  nh.param("useEKF", tempd, 1.0);
  useEKF_ = bool(tempd);

  Eigen::Matrix<double,30,1> temp30;
  Eigen::Matrix<double,6,1>   temp6;
  Eigen::Matrix<double,3,1>   temp3;
  Eigen::Matrix<double,2,1>   temp2;
  double                      temp;

  nh.param("RImu/a/x",     temp6(0), 5.0);
  nh.param("RImu/a/y",     temp6(1), 5.0);
  nh.param("RImu/a/z",     temp6(2), 5.0);
  nh.param("RImu/Omega/x", temp6(3), 1.0);
  nh.param("RImu/Omega/y", temp6(4), 1.0);
  nh.param("RImu/Omega/z", temp6(5), 1.0);
  consts_.RImu = (temp6.template cast<flt>()).asDiagonal();

  std::vector<flt> Qx_temp(NUM_STATES_TANGENT);
  nh.param("Qx",Qx_temp, std::vector<flt>(NUM_STATES_TANGENT));
  consts_.Qx = Eigen::Map<Eigen::Matrix<flt,NUM_STATES_TANGENT,1> >(&Qx_temp[0]).asDiagonal();

  nh.param("x0/I/x", temp3(0), 1.0*1e-3);
  nh.param("x0/I/y", temp3(1), 1.0*1e-3);
  nh.param("x0/I/z", temp3(2), 1.0*1e-3);
  init_vars_.I = (temp3.cast<flt>()).asDiagonal();
  nh.param("x0/m",      init_vars_.m, 0.5);
  nh.param("x0/r_BM/x", temp3(0), 0.0);
  nh.param("x0/r_BM/y", temp3(1), 0.0);
  nh.param("x0/r_BM/z", temp3(2), 0.0);
  init_vars_.r_BM = temp3.cast<flt>();
  nh.param("x0/r_BP/x", temp3(0), 0.0);
  nh.param("x0/r_BP/y", temp3(1), 0.0);
  nh.param("x0/r_BP/z", temp3(2), 0.0);
  init_vars_.r_BP = temp3.cast<flt>();
  nh.param("x0/r_BI/x", temp2(0), 0.0);
  nh.param("x0/r_BI/y", temp2(1), 0.0);
  init_vars_.r_BI = temp2.cast<flt>();

  nh.param("P0/v/x",       temp30(6), 1.0*1e-3);
  nh.param("P0/v/y",       temp30(7), 1.0*1e-3);
  nh.param("P0/v/z",       temp30(8), 1.0*1e-3);
  nh.param("P0/m",         temp30(12), 0.5);
  nh.param("P0/I/x",       temp30(13), 1.0*1e-6);
  nh.param("P0/I/y",       temp30(14), 1.0*1e-6);
  nh.param("P0/I/z",       temp30(15), 1.0*1e-6);
  nh.param("P0/r_BM/x",    temp30(16), 0.1);
  nh.param("P0/r_BM/y",    temp30(17), 0.1);
  nh.param("P0/r_BM/z",    temp30(18), 0.1);
  nh.param("P0/r_BP/x",    temp30(19), 0.1);
  nh.param("P0/r_BP/y",    temp30(20), 0.1);
  nh.param("P0/r_BP/z",    temp30(21), 0.1);
  nh.param("P0/r_BI/x",    temp30(22), 0.1);
  nh.param("P0/r_BI/y",    temp30(23), 0.1);
  nh.param("P0/b_a/x",     temp30(24), 4.0);
  nh.param("P0/b_a/y",     temp30(25), 4.0);
  nh.param("P0/b_a/z",     temp30(26), 4.0);
  nh.param("P0/b_Omega/x", temp30(27), 1.0);
  nh.param("P0/b_Omega/y", temp30(28), 1.0);
  nh.param("P0/b_Omega/z", temp30(29), 1.0);
  init_vars_.P0 = (temp30.cast<flt>()).asDiagonal();

  nh.param("UKF/alpha", temp, 0.4);
  consts_.alpha = flt(temp);
  nh.param("UKF/beta", temp, 2.0);
  consts_.beta = flt(temp);
  nh.param("UKF/kappa", temp, 0.0);
  consts_.kappa = flt(temp);


  // calcuate matrices for force & moment calculation for the case of M & B coincide
  std::vector<flt> R_temp,t_temp;
  mat3 R;
  vec3 t;
  double k_f, k_M;
  int num_rotors;
  Eigen::Matrix<flt,3,20> P_f,P_M;
  for (num_rotors = 0; num_rotors < 20; num_rotors++)
  {
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

  consts_.H_cov_poseUpdate = Eigen::Matrix<flt,NUM_MEAS_POSE,NUM_STATES>::Zero();
  consts_.H_cov_poseUpdate.block(0,0,3,3) = consts_.eye3; // r
  consts_.H_cov_poseUpdate.block(3,9,3,3) = consts_.eye3; // Omega
}

void InertiaEstimator::initializeFilter(const InitVars &init)
{
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

void InertiaEstimator::predictEKF(StateWithCov &state, Input &input)
{
  flt dt = (input.t-state.t).toSec();
  if (dt > 1.0e-5)
  {
    angleA orientation = qLog(state.X.q)*2.0;

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
    // us with floats
    //Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> F(F_temp);

    state.t = input.t;
    processModel(state.X,input,dt);
    state.P = F*state.P*F.transpose() + consts_.Qx*dt;
  }
}

void InertiaEstimator::predictUKF(StateWithCov &state, Input &input)
{
  flt dt = (input.t - state.t).toSec();

  if (dt > 1.0e-5) {
    // create and propagate sigma points
    State sigmaPoints[NUM_SIGMAS];
    //Eigen::Matrix<flt,NUM_STATES,NUM_SIGMAS> propagatedSigmaPointsMat;
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
    state.t = input.t;
    //state.X = stateVec2state(mu);
    state.P = diff*consts_.Wc.asDiagonal()*diff.transpose() + consts_.Qx*dt;
  }
}

void InertiaEstimator::measUpdatePoseEKF(StateWithCov &state, MeasPose &meas)
{
  // linear update for orientation measurement
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H.block(0,0,3,3) = mat3::Identity();

  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = qBoxMinus(meas.q,state.X.q);

  Eigen::Matrix<flt,3,3> S_KF = meas.cov.block(0,0,3,3) + H*state.P*H.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H.transpose() * S_KF.inverse();

  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;

  boxplus(state.X,dX);
  state.P = (consts_.eye_NST-K_KF*H)*state.P*(consts_.eye_NST-K_KF*H).transpose() + K_KF*meas.cov.block(0,0,3,3)*K_KF.transpose();

  // non-linear update for position measurement
  angleA orientation = qLog(state.X.q)*2.0;
  double H_temp[90];

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

  vec3 zpred = measurementModelPose(state.X);
  mat3 S_EKF = H_EKF*state.P*H_EKF.transpose() + meas.cov.block(3,3,3,3);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_EKF = state.P*H_EKF.transpose()*S_EKF.inverse();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_EKF*(meas.r - zpred);

  boxplus(state.X,del);
  state.P = (Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity() - K_EKF*H_EKF) * state.P;
}

void InertiaEstimator::measUpdatePoseUKF(StateWithCov &state, MeasPose &meas)
{
  // linear update for orientation measurement
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H.block(0,0,3,3) = mat3::Identity();

  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = qBoxMinus(meas.q,state.X.q);

  Eigen::Matrix<flt,3,3> S_KF = meas.cov.block(0,0,3,3) + H*state.P*H.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H.transpose() * S_KF.inverse();

  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;

  boxplus(state.X,dX);
  state.P = (consts_.eye_NST-K_KF*H)*state.P*(consts_.eye_NST-K_KF*H).transpose() + K_KF*meas.cov.block(0,0,3,3)*K_KF.transpose();

  // Unscented Kalman Filter update step for position measurement
  const int L = 2*(NUM_STATES_TANGENT)+1;
  State sigmaPoints[L];
  Eigen::Matrix<flt,3,L> propagatedSigmaPointsMat;
  getSigmaPoints(sigmaPoints,state);

  for (int i = 0; i<L; i++)
    propagatedSigmaPointsMat.col(i) = measurementModelPose(sigmaPoints[i]);

  // calculate the mean
  vec3 zhat = propagatedSigmaPointsMat*consts_.Wm;

  Eigen::Matrix<flt,3,L> diffZ;
  diffZ = propagatedSigmaPointsMat - zhat.replicate(1,L);

  // calculate the covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,L> diffX;
  for (int i=0; i<L; i++) {
    diffX.col(i) = sigmaPoints[i].boxminus(state.X);
  }

  mat3 S_UKF  = diffZ * consts_.Wc.asDiagonal() * diffZ.transpose() + meas.cov.block(3,3,3,3);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> Pcross = diffX * consts_.Wc.asDiagonal() * diffZ.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_UKF = Pcross*S_UKF.inverse();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_UKF*(meas.r - zhat);

  boxplus(state.X,del);
  state.P = state.P - K_UKF*S_UKF*K_UKF.transpose();
}

void InertiaEstimator::measUpdateImuEKF(StateWithCov &state, Input &input, MeasImu &meas)
{
  // linear update for angular velocity measurement
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_P = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H_P.block(0,9,3,3) = mat3::Identity();
  H_P.block(0,27,3,3) = mat3::Identity();

  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = meas.Omega - (state.X.Omega + state.X.b_Omega);

  Eigen::Matrix<flt,3,3> S_KF = H_P*state.P*H_P.transpose() + consts_.RImu.block(3,3,3,3);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H_P.transpose() * S_KF.inverse();

  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;

  boxplus(state.X,dX);
  state.P = (consts_.eye_NST-K_KF*H_P)*state.P*(consts_.eye_NST-K_KF*H_P).transpose() + K_KF*consts_.RImu.block(3,3,3,3)*K_KF.transpose();

  // Extended Kalman Filter update step for acceleration measurement
  angleA orientation = qLog(state.X.q)*2.0;
  flt dt = (input.t-state.t).toSec();
  double H_temp[90];

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

  vec3 zpred = measurementModelImu(state.X,input);
  vec3 yhat = meas.a - zpred;
  mat3 S_EKF = H_EKF*state.P*H_EKF.transpose() + consts_.RImu.block(0,0,3,3);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_EKF = state.P*H_EKF.transpose()*(S_EKF.inverse());
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_EKF*yhat;

  boxplus(state.X,del);
  state.P = ( Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>::Identity() - K_EKF*H_EKF) * state.P;
}

void InertiaEstimator::measUpdateImuUKF(StateWithCov &state, Input &input, MeasImu &meas)
{
  // linear update for angular velocity measurement
  Eigen::Matrix<flt,3,NUM_STATES_TANGENT> H_P = Eigen::Matrix<flt,3,NUM_STATES_TANGENT>::Zero();
  H_P.block(0,9,3,3) = mat3::Identity();
  H_P.block(0,27,3,3) = mat3::Identity();

  Eigen::Matrix<flt,3,1> Xhat;
  Xhat = meas.Omega - (state.X.Omega + state.X.b_Omega);

  Eigen::Matrix<flt,3,3> S_KF = consts_.RImu.block(3,3,3,3) + H_P*state.P*H_P.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_KF = state.P * H_P.transpose() * S_KF.inverse();

  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> dX = K_KF*Xhat;

  boxplus(state.X,dX);
  state.P = (consts_.eye_NST-K_KF*H_P)*state.P*(consts_.eye_NST-K_KF*H_P).transpose() + K_KF*consts_.RImu.block(3,3,3,3)*K_KF.transpose();

  // Unscented Kalman Filter update step for acceleration measurement
  const int L = 2*(NUM_STATES_TANGENT)+1;
  State sigmaPoints[L];
  Eigen::Matrix<flt,3,L> propagatedSigmaPointsMat;
  getSigmaPoints(sigmaPoints,state);

  for (int i = 0; i<L; i++)
    propagatedSigmaPointsMat.col(i) = measurementModelImu(sigmaPoints[i],input);

  // calculate the mean
  vec3 zhat = propagatedSigmaPointsMat*consts_.Wm;

  Eigen::Matrix<flt,3,L> diffZ;
  diffZ = propagatedSigmaPointsMat - zhat.replicate(1,L);

  // calculate the covariance
  Eigen::Matrix<flt,NUM_STATES_TANGENT,L> diffX;
  for (int i=0; i<L; i++) {
    diffX.col(i) = sigmaPoints[i].boxminus(state.X);
  }

  mat3 S_UKF  = diffZ * consts_.Wc.asDiagonal() * diffZ.transpose() + consts_.RImu.block(0,0,3,3);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> Pcross = diffX * consts_.Wc.asDiagonal() * diffZ.transpose();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,3> K_UKF = Pcross*S_UKF.inverse();
  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> del = K_UKF*(meas.a - zhat);

  boxplus(state.X,del);
  state.P = state.P - K_UKF*S_UKF*K_UKF.transpose();
}




void InertiaEstimator::processModel(State &X, Input &U, flt &dt)
{
  //static Eigen::Matrix<flt,3,Eigen::Dynamic> correction;
  static Eigen::Matrix<flt,3,4> P_M_CoGcorrected;
  P_M_CoGcorrected = consts_.P_M + consts_.k_f*Eigen::Matrix<flt,3,1>(-X.r_BM(1),X.r_BM(0),0.0).replicate(1,consts_.num_rotors);

  Eigen::Matrix<flt,NUM_STATES_TANGENT,1> delta;
  delta <<  X.Omega*dt,
            X.v*dt,
            (1.0/X.m*qRotateVec(X.q,consts_.P_f*U.rpm_sq) - consts_.g*consts_.e_z)*dt,
            (X.I.inverse()*((P_M_CoGcorrected*U.rpm_sq)-X.Omega.cross(X.I*X.Omega)))*dt,
            0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,
            0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f;
      //Eigen::Matrix<flt,NUM_STATES_TANGENT-3*4,1>::Zero(); // all other state stay const
  boxplus(X,delta);
}

Eigen::Matrix<flt,3,1> InertiaEstimator::measurementModelPose(State &X)
{

  Eigen::Matrix<flt,3,1> meas = X.r + qRotateVec(X.q,X.r_BP-X.r_BM);
  return meas;
}

Eigen::Matrix<flt,3,1> InertiaEstimator::measurementModelImu(State &X, Input &U)
{
  vec3 r_MI = vec3(X.r_BI.x(),X.r_BI.y(),0) - X.r_BM;

  static Eigen::Matrix<flt,3,Eigen::Dynamic> P_M_CoGcorrected;
  P_M_CoGcorrected = consts_.P_M + consts_.k_f*Eigen::Matrix<flt,3,1>(-X.r_BM(1),X.r_BM(0),0.0).replicate(1,consts_.num_rotors);

  vec3 F = consts_.P_f*     U.rpm_sq;
  vec3 M = P_M_CoGcorrected*U.rpm_sq;

  vec3 Omega_dot = X.I.inverse()*(M-X.Omega.cross(X.I*X.Omega));

  vec3 a_meas = 1.0/X.m*F
                + Omega_dot.cross(r_MI)
                + X.Omega.cross(X.Omega.cross(r_MI))
                + X.b_a;

  return a_meas;
}




void InertiaEstimator::getSigmaPoints(State *sigmaPoints, StateWithCov &state)
{
  // Cholesky factorization
  Eigen::LLT<Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT>> factorization(state.P);
  Eigen::Matrix<flt,NUM_STATES_TANGENT,NUM_STATES_TANGENT> sqrt = factorization.matrixL();
  flt c = std::sqrt(consts_.lambda + NUM_STATES_TANGENT);
  sqrt *= c;

  sigmaPoints[0] = state.X;
  for (int i = 1; i <= NUM_STATES_TANGENT; i++){
    sigmaPoints[i]                    = state.X.boxplus(sqrt.col(i-1));
    sigmaPoints[i+NUM_STATES_TANGENT] = state.X.boxplus(-sqrt.col(i-1));
  }
}

void InertiaEstimator::calculateMeanStateSigma(State &mean, const State sigmaPoints[NUM_SIGMAS])
{
  quat qList[NUM_SIGMAS];
  for (int i = 0; i < NUM_SIGMAS; i++)
    qList[i] = sigmaPoints[i].q;
  mean.q = qMean(qList,consts_.Wm);

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




//void InertiaEstimator::rpm_callback(const quadrotor_msgs::MotorRPM::ConstPtr &msg)
void InertiaEstimator::rpm_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  if (initialized_)
  {
    RPM_skip_counter_++;
    RPM_skip_counter_ = RPM_skip_counter_ % 5;
    if ( RPM_skip_counter_ == 0 )
    {
      rpmMsg2input(input_,msg);
      if (useEKF_)
        predictEKF(state_, input_);
      else
        predictUKF(state_, input_);
      publishEstimates(state_);
    }
  }
  else
  {
    rpmMsg2input(input_,msg);
    if (inputList_.size() >= 30)
      inputList_.pop();
    init_vars_.rpmInitialized = true;
    init_vars_.t = msg->header.stamp;
    init_vars_.readyToInitialize = init_vars_.rpmInitialized &&
                                   init_vars_.imuInitialized &&
                                   init_vars_.poseInitialized;

    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

void InertiaEstimator::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  MeasPose meas_pose = poseMsg2measPose(msg);
  if (initialized_)
  {
    if (useEKF_)
      measUpdatePoseEKF(state_, meas_pose);
    else
      measUpdatePoseUKF(state_, meas_pose);
    publishEstimates(state_);
  }
  else
  {
    init_vars_.r   = meas_pose.r;
    init_vars_.q   = meas_pose.q;
    init_vars_.P0.block(0,0,3,3) = meas_pose.cov.block(0,0,3,3);
    init_vars_.P0.block(3,3,3,3) = meas_pose.cov.block(3,3,3,3);
    init_vars_.poseInitialized = true;
    init_vars_.readyToInitialize = init_vars_.rpmInitialized &&
                                   init_vars_.imuInitialized &&
                                   init_vars_.poseInitialized;

    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

void InertiaEstimator::imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
  if (initialized_)
  {
    IMU_skip_counter_++;
    IMU_skip_counter_ = IMU_skip_counter_ % 5;
    if ( IMU_skip_counter_ == 0 )
    {
      MeasImu meas = imuMsg2measImu(msg);
      if (useEKF_)
        measUpdateImuEKF(state_,input_,meas);
      else
        measUpdateImuUKF(state_,input_,meas);
      publishEstimates(state_);
    }
  }
  else
  {
    MeasImu meas_imu;
    meas_imu = imuMsg2measImu(msg);
    init_vars_.Omega = meas_imu.Omega;
    init_vars_.P0.block(9,9,3,3) = consts_.RImu.block(3,3,3,3);
    init_vars_.imuInitialized = true;
    init_vars_.readyToInitialize = init_vars_.rpmInitialized &&
                                   init_vars_.imuInitialized &&
                                   init_vars_.poseInitialized;

    if (init_vars_.readyToInitialize)
      initializeFilter(init_vars_);
  }
}

void InertiaEstimator::rpmMsg2input(Input &returnVar, const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  returnVar.rpm_sq << msg->wrench.force.x*msg->wrench.force.x,
                      msg->wrench.force.y*msg->wrench.force.y,
                      msg->wrench.force.z*msg->wrench.force.z,
                      msg->wrench.torque.x*msg->wrench.torque.x;
  returnVar.t = msg->header.stamp;
}

MeasPose InertiaEstimator:: poseMsg2measPose(const nav_msgs::Odometry::ConstPtr &msg)
{
  MeasPose meas_pose;
  meas_pose.r = vec3(msg->pose.pose.position.x,
                    -msg->pose.pose.position.y,
                    -msg->pose.pose.position.z);
  meas_pose.q = quat(msg->pose.pose.orientation.w,
                     msg->pose.pose.orientation.x,
                    -msg->pose.pose.orientation.y,
                    -msg->pose.pose.orientation.z).normalized();

  // rotate s.t. z is up instead of down
  mat6 covPose;
  getCovInMsg(msg->pose.covariance,covPose);
  covPose = consts_.rotate6*covPose*consts_.rotate6;

  meas_pose.cov = covPose;

  return meas_pose;
}

MeasImu InertiaEstimator::imuMsg2measImu(const sensor_msgs::Imu::ConstPtr &msg)
{
  MeasImu meas_imu;

  meas_imu.a     = vec3(msg->linear_acceleration.x,
                        -msg->linear_acceleration.y,
                        -msg->linear_acceleration.z);

  meas_imu.Omega = vec3(msg->angular_velocity.x,
                        -msg->angular_velocity.y,
                        -msg->angular_velocity.z);

  return meas_imu;
}

void InertiaEstimator::publishEstimates(const StateWithCov &estimate)
{
  static inertia_estimator::ParameterEstimates msg;
  nav_msgs::Odometry t;

  msg.header.stamp = estimate.t;

  msg.q = quat2msg(estimate.X.q);
  msg.r = vec2msg(estimate.X.r);
  msg.v = vec2msg(estimate.X.v);
  msg.Omega = vec2msg(estimate.X.Omega);

  msg.m    = estimate.X.m;
  msg.I    = vec2msg(estimate.X.I.diagonal());
  msg.r_BM = vec2msg(estimate.X.r_BM);

  msg.r_BP    = vec2msg(estimate.X.r_BP);
  msg.r_BI.x = estimate.X.r_BI(0);
  msg.r_BI.y = estimate.X.r_BI(1);
  msg.b_a     = vec2msg(estimate.X.b_a);
  msg.b_Omega = vec2msg(estimate.X.b_Omega);

  for (int row = 0; row < NUM_STATES_TANGENT; row++)
    msg.covariance[row] = estimate.P(row,row);

  pub_estimates_.publish(msg);
}




Eigen::Matrix<flt,NUM_STATES,1> InertiaEstimator::state2stateVec(const State &X)
{
  Eigen::Matrix<flt,NUM_STATES,1> stateVec;
  stateVec << X.q.w(),X.q.x(),X.q.y(),X.q.z(), X.r, X.v, X.Omega, X.m, X.I.diagonal(), X.r_BM, X.r_BP, X.r_BI, X.b_a, X.b_Omega;
  return stateVec;
}

State InertiaEstimator::stateVec2state(const Eigen::Matrix<flt,NUM_STATES,1> &XV)
{
  State X;
  X.q = quat(XV(0,0),XV(1,0),XV(2,0),XV(3,0));
  X.r = XV.block(4,0,3,1);
  X.v = XV.block(7,0,3,1);
  X.Omega = XV.block(10,0,3,1);
  X.m = XV(13,0);
  X.I = XV.block(14,0,3,1).asDiagonal();
  X.r_BM = XV.block(17,0,3,1);
  X.r_BP = XV.block(20,0,3,1);
  X.r_BI = XV.block(23,0,2,1);
  X.b_a = XV.block(25,0,3,1);
  X.b_Omega = XV.block(28,0,3,1);

  return X;
}

void InertiaEstimator::reset()
{
  init_vars_ = InitVars();
  initialized_ = false;
  warn("reset",int(init_vars_.rpmInitialized || init_vars_.imuInitialized || init_vars_.poseInitialized));
}



void InertiaEstimator::boxplus(State &X, const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta)
{
  X.q       = qBoxPlus(X.q,delta.segment(0,3));
  X.r       += delta.segment(3,3);
  X.v       += delta.segment(6,3);
  X.Omega   += delta.segment(9,3);

  X.m       += delta(12);
  //vec3 temp(delta.segment(13,3));
  mat3 temp2(delta.segment(13,3).asDiagonal());
  X.I       += temp2;
  X.r_BM    += delta.segment(16,3);
  X.r_BP    += delta.segment(19,3);
  X.r_BI    += delta.segment(22,2);
  X.b_a     += delta.segment(24,3);
  X.b_Omega += delta.segment(27,3);
}

State State::boxplus(const Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> &delta)
{
  State sum;

  sum.q       = qBoxPlus(this->q,delta.segment(0,3));
  sum.r       = this->r       + delta.segment(3,3);
  sum.v       = this->v       + delta.segment(6,3);
  sum.Omega   = this->Omega   + delta.segment(9,3);

  sum.m       = this->m       + delta(12);
  //vec3 temp(delta.segment(13,3));
  mat3 temp2(delta.segment(13,3).asDiagonal());
  sum.I       = this->I       + temp2;
  sum.r_BM    = this->r_BM    + delta.segment(16,3);
  sum.r_BP    = this->r_BP    + delta.segment(19,3);
  sum.r_BI    = this->r_BI    + delta.segment(22,2);
  sum.b_a     = this->b_a     + delta.segment(24,3);
  sum.b_Omega = this->b_Omega + delta.segment(27,3);

  return sum;
}

Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> State::boxminus(const State &other) const
{
  Eigen::Matrix<flt, NUM_STATES_TANGENT, 1> delta;

  delta.segment(0,3)  = qBoxMinus(this->q,other.q);
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

