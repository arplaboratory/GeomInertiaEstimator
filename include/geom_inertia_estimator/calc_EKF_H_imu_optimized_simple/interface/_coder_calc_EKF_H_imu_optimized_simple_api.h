/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_calc_EKF_H_imu_optimized_simple_api.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 26-Feb-2019 22:45:59
 */

#ifndef _CODER_CALC_EKF_H_IMU_OPTIMIZED_SIMPLE_API_H
#define _CODER_CALC_EKF_H_IMU_OPTIMIZED_SIMPLE_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_calc_EKF_H_imu_optimized_simple_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void calc_EKF_H_imu_optimized_simple(real_T qLx, real_T qLy, real_T qLz,
  real_T rx, real_T ry, real_T rz, real_T vx, real_T vy, real_T vz, real_T
  Omegax, real_T Omegay, real_T Omegaz, real_T mtot, real_T Itotxx, real_T
  Itotyy, real_T Itotzz, real_T t_GeomCogX, real_T t_GeomCogY, real_T t_GeomCogZ,
  real_T t_GeomOdomX, real_T t_GeomOdomY, real_T t_GeomOdomZ, real_T t_GeomImuX,
  real_T t_GeomImuY, real_T ImuBiasAccX, real_T ImuBiasAccY, real_T ImuBiasAccZ,
  real_T ImuBiasAngVelX, real_T ImuBiasAngVelY, real_T ImuBiasAngVelZ, real_T
  motor1, real_T motor2, real_T motor3, real_T motor4, real_T dt, real_T
  t_GeomImuZ, real_T w, real_T l, real_T c, real_T k_f, real_T Himu[90]);
extern void calc_EKF_H_imu_optimized_simple_api(const mxArray * const prhs[40],
  const mxArray *plhs[1]);
extern void calc_EKF_H_imu_optimized_simple_atexit(void);
extern void calc_EKF_H_imu_optimized_simple_initialize(void);
extern void calc_EKF_H_imu_optimized_simple_terminate(void);
extern void calc_EKF_H_imu_optimized_simple_xil_terminate(void);

#endif

/*
 * File trailer for _coder_calc_EKF_H_imu_optimized_simple_api.h
 *
 * [EOF]
 */
