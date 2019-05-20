/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_calc_EKF_H_odom_optimized_simple_mex.cpp
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 26-Feb-2019 22:48:03
 */

/* Include Files */
#include "_coder_calc_EKF_H_odom_optimized_simple_api.h"
#include "_coder_calc_EKF_H_odom_optimized_simple_mex.h"

/* Function Declarations */
static void c_calc_EKF_H_odom_optimized_sim(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[40]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                const mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[40]
 * Return Type  : void
 */
static void c_calc_EKF_H_odom_optimized_sim(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[40])
{
  const mxArray *inputs[40];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 40) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 40, 4,
                        32, "calc_EKF_H_odom_optimized_simple");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 32,
                        "calc_EKF_H_odom_optimized_simple");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  calc_EKF_H_odom_optimized_simple_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  calc_EKF_H_odom_optimized_simple_terminate();
}

/*
 * Arguments    : int32_T nlhs
 *                const mxArray * const plhs[]
 *                int32_T nrhs
 *                const mxArray * const prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(calc_EKF_H_odom_optimized_simple_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  calc_EKF_H_odom_optimized_simple_initialize();

  /* Dispatch the entry-point. */
  c_calc_EKF_H_odom_optimized_sim(nlhs, plhs, nrhs, prhs);
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_calc_EKF_H_odom_optimized_simple_mex.cpp
 *
 * [EOF]
 */
