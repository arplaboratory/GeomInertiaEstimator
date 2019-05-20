/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_calc_EKF_F_optimized_api.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 26-Feb-2019 22:17:41
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_calc_EKF_F_optimized_api.h"
#include "_coder_calc_EKF_F_optimized_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131451U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "calc_EKF_F_optimized",              /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *qLx, const
  char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u[900]);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *qLx
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *qLx, const
  char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(qLx), &thisId);
  emlrtDestroyArray(&qLx);
  return y;
}

/*
 * Arguments    : const real_T u[900]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[900])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  static const int32_T iv1[2] = { 30, 30 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[2])&iv1[0], 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[40]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void calc_EKF_F_optimized_api(const mxArray * const prhs[40], const mxArray
  *plhs[1])
{
  real_T (*F)[900];
  real_T qLx;
  real_T qLy;
  real_T qLz;
  real_T rx;
  real_T ry;
  real_T rz;
  real_T vx;
  real_T vy;
  real_T vz;
  real_T Omegax;
  real_T Omegay;
  real_T Omegaz;
  real_T mtot;
  real_T Itotxx;
  real_T Itotyy;
  real_T Itotzz;
  real_T t_GeomCogX;
  real_T t_GeomCogY;
  real_T t_GeomCogZ;
  real_T t_GeomOdomX;
  real_T t_GeomOdomY;
  real_T t_GeomOdomZ;
  real_T t_GeomImuX;
  real_T t_GeomImuY;
  real_T ImuBiasAccX;
  real_T ImuBiasAccY;
  real_T ImuBiasAccZ;
  real_T ImuBiasAngVelX;
  real_T ImuBiasAngVelY;
  real_T ImuBiasAngVelZ;
  real_T motor1;
  real_T motor2;
  real_T motor3;
  real_T motor4;
  real_T dt;
  real_T t_GeomImuZ;
  real_T w;
  real_T l;
  real_T c;
  real_T k_f;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  F = (real_T (*)[900])mxMalloc(sizeof(real_T [900]));

  /* Marshall function inputs */
  qLx = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "qLx");
  qLy = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "qLy");
  qLz = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]), "qLz");
  rx = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[3]), "rx");
  ry = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "ry");
  rz = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[5]), "rz");
  vx = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[6]), "vx");
  vy = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[7]), "vy");
  vz = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[8]), "vz");
  Omegax = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[9]), "Omegax");
  Omegay = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[10]),
    "Omegay");
  Omegaz = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[11]),
    "Omegaz");
  mtot = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[12]), "mtot");
  Itotxx = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[13]),
    "Itotxx");
  Itotyy = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[14]),
    "Itotyy");
  Itotzz = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[15]),
    "Itotzz");
  t_GeomCogX = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[16]),
    "t_GeomCogX");
  t_GeomCogY = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[17]),
    "t_GeomCogY");
  t_GeomCogZ = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[18]),
    "t_GeomCogZ");
  t_GeomOdomX = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[19]),
    "t_GeomOdomX");
  t_GeomOdomY = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[20]),
    "t_GeomOdomY");
  t_GeomOdomZ = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[21]),
    "t_GeomOdomZ");
  t_GeomImuX = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[22]),
    "t_GeomImuX");
  t_GeomImuY = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[23]),
    "t_GeomImuY");
  ImuBiasAccX = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[24]),
    "ImuBiasAccX");
  ImuBiasAccY = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[25]),
    "ImuBiasAccY");
  ImuBiasAccZ = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[26]),
    "ImuBiasAccZ");
  ImuBiasAngVelX = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[27]),
    "ImuBiasAngVelX");
  ImuBiasAngVelY = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[28]),
    "ImuBiasAngVelY");
  ImuBiasAngVelZ = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[29]),
    "ImuBiasAngVelZ");
  motor1 = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[30]),
    "motor1");
  motor2 = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[31]),
    "motor2");
  motor3 = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[32]),
    "motor3");
  motor4 = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[33]),
    "motor4");
  dt = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[34]), "dt");
  t_GeomImuZ = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[35]),
    "t_GeomImuZ");
  w = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[36]), "w");
  l = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[37]), "l");
  c = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[38]), "c");
  k_f = emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[39]), "k_f");

  /* Invoke the target function */
  calc_EKF_F_optimized(qLx, qLy, qLz, rx, ry, rz, vx, vy, vz, Omegax, Omegay,
                       Omegaz, mtot, Itotxx, Itotyy, Itotzz, t_GeomCogX,
                       t_GeomCogY, t_GeomCogZ, t_GeomOdomX, t_GeomOdomY,
                       t_GeomOdomZ, t_GeomImuX, t_GeomImuY, ImuBiasAccX,
                       ImuBiasAccY, ImuBiasAccZ, ImuBiasAngVelX, ImuBiasAngVelY,
                       ImuBiasAngVelZ, motor1, motor2, motor3, motor4, dt,
                       t_GeomImuZ, w, l, c, k_f, *F);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*F);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calc_EKF_F_optimized_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  calc_EKF_F_optimized_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calc_EKF_F_optimized_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void calc_EKF_F_optimized_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_calc_EKF_F_optimized_api.c
 *
 * [EOF]
 */
