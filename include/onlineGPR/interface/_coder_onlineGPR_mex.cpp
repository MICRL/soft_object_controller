/*
 * File: _coder_onlineGPR_mex.cpp
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 10-Aug-2017 21:27:39
 */

/* Include Files */
#include "_coder_onlineGPR_api.h"
#include "_coder_onlineGPR_mex.h"

/* Function Declarations */
static void onlineGPR_mexFunction(int32_T nlhs, mxArray *plhs[6], int32_T nrhs,
  const mxArray *prhs[7]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                const mxArray *plhs[6]
 *                int32_T nrhs
 *                const mxArray *prhs[7]
 * Return Type  : void
 */
static void onlineGPR_mexFunction(int32_T nlhs, mxArray *plhs[6], int32_T nrhs,
  const mxArray *prhs[7])
{
  int32_T n;
  const mxArray *inputs[7];
  const mxArray *outputs[6];
  int32_T b_nlhs;

  /* Check for proper number of arguments. */
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal, "EMLRT:runTime:WrongNumberOfInputs",
                        5, 12, 7, 4, 9, "onlineGPR");
  }

  if (nlhs > 6) {
    emlrtErrMsgIdAndTxt(emlrtRootTLSGlobal,
                        "EMLRT:runTime:TooManyOutputArguments", 3, 4, 9,
                        "onlineGPR");
  }

  /* Temporary copy for mex inputs. */
  for (n = 0; n < nrhs; n++) {
    inputs[n] = prhs[n];
  }

  /* Call the function. */
  onlineGPR_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  onlineGPR_terminate();
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
  mexAtExit(onlineGPR_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  onlineGPR_initialize();

  /* Dispatch the entry-point. */
  onlineGPR_mexFunction(nlhs, plhs, nrhs, prhs);
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
 * File trailer for _coder_onlineGPR_mex.cpp
 *
 * [EOF]
 */
