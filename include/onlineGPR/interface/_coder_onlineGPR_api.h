/*
 * File: _coder_onlineGPR_api.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 10-Aug-2017 21:27:39
 */

#ifndef _CODER_ONLINEGPR_API_H
#define _CODER_ONLINEGPR_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_onlineGPR_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void onlineGPR(real_T x[6], real_T y[6], real_T x_star[6], real_T
                      gram_matrix_data[], int32_T gram_matrix_size[2], real_T
                      X_data[], int32_T X_size[2], real_T Y_data[], int32_T
                      Y_size[2], real_T K_data[], int32_T K_size[2], real_T
                      gram_matrix_update_data[], int32_T
                      gram_matrix_update_size[2], real_T y_star[6], real_T
                      *cov_star, real_T X_update_data[], int32_T X_update_size[2],
                      real_T Y_update_data[], int32_T Y_update_size[2], real_T
                      K_update_data[], int32_T K_update_size[2]);
extern void onlineGPR_api(const mxArray *prhs[7], const mxArray *plhs[6]);
extern void onlineGPR_atexit(void);
extern void onlineGPR_initialize(void);
extern void onlineGPR_terminate(void);
extern void onlineGPR_xil_terminate(void);

#endif

/*
 * File trailer for _coder_onlineGPR_api.h
 *
 * [EOF]
 */
