//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: calc_EKF_F_optimized.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 26-Feb-2019 22:17:41
//
#ifndef CALC_EKF_F_OPTIMIZED_H
#define CALC_EKF_F_OPTIMIZED_H

// Include Files
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "calc_EKF_F_optimized_types.h"

// Function Declarations
extern void calc_EKF_F_optimized(double qLx, double qLy, double qLz, double rx,
  double ry, double rz, double vx, double vy, double vz, double Omegax, double
  Omegay, double Omegaz, double mtot, double Itotxx, double Itotyy, double
  Itotzz, double t_GeomCogX, double t_GeomCogY, double t_GeomCogZ, double
  t_GeomOdomX, double t_GeomOdomY, double t_GeomOdomZ, double t_GeomImuX, double
  t_GeomImuY, double ImuBiasAccX, double ImuBiasAccY, double ImuBiasAccZ, double
  ImuBiasAngVelX, double ImuBiasAngVelY, double ImuBiasAngVelZ, double motor1,
  double motor2, double motor3, double motor4, double dt, double t_GeomImuZ,
  double w, double l, double c, double k_f, double F[900]);

#endif

//
// File trailer for calc_EKF_F_optimized.h
//
// [EOF]
//
