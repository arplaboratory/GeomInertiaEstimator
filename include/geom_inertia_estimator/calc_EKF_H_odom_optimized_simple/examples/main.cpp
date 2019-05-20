//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 26-Feb-2019 22:48:03
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "calc_EKF_H_odom_optimized_simple.h"
#include "main.h"
#include "calc_EKF_H_odom_optimized_simple_terminate.h"
#include "calc_EKF_H_odom_optimized_simple_initialize.h"

// Function Declarations
static double argInit_real_T();
static void main_calc_EKF_H_odom_optimized_simple();

// Function Definitions

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_calc_EKF_H_odom_optimized_simple()
{
  double qLx;
  double qLy;
  double qLz;
  double rx;
  double ry;
  double rz;
  double vx;
  double vy;
  double vz;
  double Omegax;
  double Omegay;
  double Omegaz;
  double mtot;
  double Itotxx;
  double Itotyy;
  double Itotzz;
  double t_GeomCogX;
  double t_GeomCogY;
  double t_GeomCogZ;
  double t_GeomOdomX;
  double t_GeomOdomY;
  double t_GeomOdomZ;
  double t_GeomImuX;
  double t_GeomImuY;
  double ImuBiasAccX;
  double Hodom[90];

  // Initialize function 'calc_EKF_H_odom_optimized_simple' input arguments.
  qLx = argInit_real_T();
  qLy = argInit_real_T();
  qLz = argInit_real_T();
  rx = argInit_real_T();
  ry = argInit_real_T();
  rz = argInit_real_T();
  vx = argInit_real_T();
  vy = argInit_real_T();
  vz = argInit_real_T();
  Omegax = argInit_real_T();
  Omegay = argInit_real_T();
  Omegaz = argInit_real_T();
  mtot = argInit_real_T();
  Itotxx = argInit_real_T();
  Itotyy = argInit_real_T();
  Itotzz = argInit_real_T();
  t_GeomCogX = argInit_real_T();
  t_GeomCogY = argInit_real_T();
  t_GeomCogZ = argInit_real_T();
  t_GeomOdomX = argInit_real_T();
  t_GeomOdomY = argInit_real_T();
  t_GeomOdomZ = argInit_real_T();
  t_GeomImuX = argInit_real_T();
  t_GeomImuY = argInit_real_T();
  ImuBiasAccX = argInit_real_T();

  // Call the entry-point 'calc_EKF_H_odom_optimized_simple'.
  calc_EKF_H_odom_optimized_simple(qLx, qLy, qLz, rx, ry, rz, vx, vy, vz, Omegax,
    Omegay, Omegaz, mtot, Itotxx, Itotyy, Itotzz, t_GeomCogX, t_GeomCogY,
    t_GeomCogZ, t_GeomOdomX, t_GeomOdomY, t_GeomOdomZ, t_GeomImuX, t_GeomImuY,
    ImuBiasAccX, argInit_real_T(), argInit_real_T(), argInit_real_T(),
    argInit_real_T(), argInit_real_T(), argInit_real_T(), argInit_real_T(),
    argInit_real_T(), argInit_real_T(), argInit_real_T(), argInit_real_T(),
    argInit_real_T(), argInit_real_T(), argInit_real_T(), argInit_real_T(),
    Hodom);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  calc_EKF_H_odom_optimized_simple_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_calc_EKF_H_odom_optimized_simple();

  // Terminate the application.
  // You do not need to do this more than one time.
  calc_EKF_H_odom_optimized_simple_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
