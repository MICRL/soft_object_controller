/*
 * File: _coder_onlineGPR_api.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 10-Aug-2017 21:27:39
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_onlineGPR_api.h"
#include "_coder_onlineGPR_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131450U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "onlineGPR",                         /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[6];
static const mxArray *b_emlrt_marshallOut(const real_T u[6]);
static real_T (*c_emlrt_marshallIn(const mxArray *y, const char_T *identifier))
  [6];
static const mxArray *c_emlrt_marshallOut(const real_T u);
static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[6];
static const mxArray *d_emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2]);
static void e_emlrt_marshallIn(const mxArray *gram_matrix, const char_T
  *identifier, real_T **y_data, int32_T y_size[2]);
static const mxArray *e_emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2]);
static real_T (*emlrt_marshallIn(const mxArray *x, const char_T *identifier))[6];
static const mxArray *emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2]);
static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T **y_data, int32_T y_size[2]);
static void g_emlrt_marshallIn(const mxArray *X, const char_T *identifier,
  real_T **y_data, int32_T y_size[2]);
static void h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T **y_data, int32_T y_size[2]);
static void i_emlrt_marshallIn(const mxArray *Y, const char_T *identifier,
  real_T **y_data, int32_T y_size[2]);
static void j_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T **y_data, int32_T y_size[2]);
static real_T (*k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[6];
static real_T (*l_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[6];
static void m_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T **ret_data, int32_T ret_size[2]);
static void n_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T **ret_data, int32_T ret_size[2]);
static void o_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T **ret_data, int32_T ret_size[2]);

/* Function Definitions */

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[6]
 */
static real_T (*b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[6]
{
  real_T (*y)[6];
  y = k_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const real_T u[6]
 * Return Type  : const mxArray *
 */
  static const mxArray *b_emlrt_marshallOut(const real_T u[6])
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv1[2] = { 0, 0 };

  static const int32_T iv2[2] = { 1, 6 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv1, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m1, *(int32_T (*)[2])&iv2[0], 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const mxArray *y
 *                const char_T *identifier
 * Return Type  : real_T (*)[6]
 */
static real_T (*c_emlrt_marshallIn(const mxArray *y, const char_T *identifier))
  [6]
{
  real_T (*b_y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_y = d_emlrt_marshallIn(emlrtAlias(y), &thisId);
  emlrtDestroyArray(&y);
  return b_y;
}
/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
  static const mxArray *c_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m2;
  y = NULL;
  m2 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[6]
 */
static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[6]
{
  real_T (*y)[6];
  y = l_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const real_T u_data[]
 *                const int32_T u_size[2]
 * Return Type  : const mxArray *
 */
  static const mxArray *d_emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2])
{
  const mxArray *y;
  const mxArray *m3;
  static const int32_T iv3[2] = { 0, 0 };

  y = NULL;
  m3 = emlrtCreateNumericArray(2, iv3, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m3, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m3, *(int32_T (*)[2])&u_size[0], 2);
  emlrtAssign(&y, m3);
  return y;
}

/*
 * Arguments    : const mxArray *gram_matrix
 *                const char_T *identifier
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void e_emlrt_marshallIn(const mxArray *gram_matrix, const char_T
  *identifier, real_T **y_data, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(emlrtAlias(gram_matrix), &thisId, y_data, y_size);
  emlrtDestroyArray(&gram_matrix);
}

/*
 * Arguments    : const real_T u_data[]
 *                const int32_T u_size[2]
 * Return Type  : const mxArray *
 */
static const mxArray *e_emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2])
{
  const mxArray *y;
  const mxArray *m4;
  static const int32_T iv4[2] = { 0, 0 };

  y = NULL;
  m4 = emlrtCreateNumericArray(2, iv4, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m4, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m4, *(int32_T (*)[2])&u_size[0], 2);
  emlrtAssign(&y, m4);
  return y;
}

/*
 * Arguments    : const mxArray *x
 *                const char_T *identifier
 * Return Type  : real_T (*)[6]
 */
static real_T (*emlrt_marshallIn(const mxArray *x, const char_T *identifier))[6]
{
  real_T (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}
/*
 * Arguments    : const real_T u_data[]
 *                const int32_T u_size[2]
 * Return Type  : const mxArray *
 */
  static const mxArray *emlrt_marshallOut(const real_T u_data[], const int32_T
  u_size[2])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)&u_data[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[2])&u_size[0], 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T **y_data, int32_T y_size[2])
{
  m_emlrt_marshallIn(emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const mxArray *X
 *                const char_T *identifier
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void g_emlrt_marshallIn(const mxArray *X, const char_T *identifier,
  real_T **y_data, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  h_emlrt_marshallIn(emlrtAlias(X), &thisId, y_data, y_size);
  emlrtDestroyArray(&X);
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T **y_data, int32_T y_size[2])
{
  n_emlrt_marshallIn(emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const mxArray *Y
 *                const char_T *identifier
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void i_emlrt_marshallIn(const mxArray *Y, const char_T *identifier,
  real_T **y_data, int32_T y_size[2])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  j_emlrt_marshallIn(emlrtAlias(Y), &thisId, y_data, y_size);
  emlrtDestroyArray(&Y);
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real_T **y_data
 *                int32_T y_size[2]
 * Return Type  : void
 */
static void j_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId, real_T **y_data, int32_T y_size[2])
{
  o_emlrt_marshallIn(emlrtAlias(u), parentId, y_data, y_size);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[6]
 */
static real_T (*k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[6]
{
  real_T (*ret)[6];
  static const int32_T dims[1] = { 6 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
    dims);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[6]
 */
  static real_T (*l_emlrt_marshallIn(const mxArray *src, const
  emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  static const int32_T dims[2] = { 1, 6 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims);
  ret = (real_T (*)[6])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T **ret_data
 *                int32_T ret_size[2]
 * Return Type  : void
 */
static void m_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T **ret_data, int32_T ret_size[2])
{
  static const int32_T dims[2] = { 100, 100 };

  const boolean_T bv0[2] = { true, true };

  int32_T iv5[2];
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims, &bv0[0], iv5);
  ret_size[0] = iv5[0];
  ret_size[1] = iv5[1];
  *ret_data = (real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T **ret_data
 *                int32_T ret_size[2]
 * Return Type  : void
 */
static void n_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T **ret_data, int32_T ret_size[2])
{
  static const int32_T dims[2] = { 6, 100 };

  const boolean_T bv1[2] = { false, true };

  int32_T iv6[2];
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims, &bv1[0], iv6);
  ret_size[0] = iv6[0];
  ret_size[1] = iv6[1];
  *ret_data = (real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real_T **ret_data
 *                int32_T ret_size[2]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId, real_T **ret_data, int32_T ret_size[2])
{
  static const int32_T dims[2] = { 100, 6 };

  const boolean_T bv2[2] = { true, false };

  int32_T iv7[2];
  emlrtCheckVsBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims, &bv2[0], iv7);
  ret_size[0] = iv7[0];
  ret_size[1] = iv7[1];
  *ret_data = (real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const mxArray *prhs[7]
 *                const mxArray *plhs[6]
 * Return Type  : void
 */
void onlineGPR_api(const mxArray *prhs[7], const mxArray *plhs[6])
{
  real_T (*gram_matrix_update_data)[10201];
  real_T (*y_star)[6];
  real_T (*X_update_data)[606];
  real_T (*Y_update_data)[606];
  real_T (*K_update_data)[10201];
  real_T (*x)[6];
  real_T (*y)[6];
  real_T (*x_star)[6];
  real_T (*gram_matrix_data)[10000];
  int32_T gram_matrix_size[2];
  real_T (*X_data)[600];
  int32_T X_size[2];
  real_T (*Y_data)[600];
  int32_T Y_size[2];
  real_T (*K_data)[10000];
  int32_T K_size[2];
  int32_T gram_matrix_update_size[2];
  real_T cov_star;
  int32_T X_update_size[2];
  int32_T Y_update_size[2];
  int32_T K_update_size[2];
  gram_matrix_update_data = (real_T (*)[10201])mxMalloc(sizeof(real_T [10201]));
  y_star = (real_T (*)[6])mxMalloc(sizeof(real_T [6]));
  X_update_data = (real_T (*)[606])mxMalloc(sizeof(real_T [606]));
  Y_update_data = (real_T (*)[606])mxMalloc(sizeof(real_T [606]));
  K_update_data = (real_T (*)[10201])mxMalloc(sizeof(real_T [10201]));
  prhs[4] = emlrtProtectR2012b(prhs[4], 4, false, 600);
  prhs[5] = emlrtProtectR2012b(prhs[5], 5, false, 600);
  prhs[6] = emlrtProtectR2012b(prhs[6], 6, false, 10000);

  /* Marshall function inputs */
  x = emlrt_marshallIn(emlrtAlias(prhs[0]), "x");
  y = c_emlrt_marshallIn(emlrtAlias(prhs[1]), "y");
  x_star = emlrt_marshallIn(emlrtAlias(prhs[2]), "x_star");
  e_emlrt_marshallIn(emlrtAlias(prhs[3]), "gram_matrix", (real_T **)
                     &gram_matrix_data, gram_matrix_size);
  g_emlrt_marshallIn(emlrtAlias(prhs[4]), "X", (real_T **)&X_data, X_size);
  i_emlrt_marshallIn(emlrtAlias(prhs[5]), "Y", (real_T **)&Y_data, Y_size);
  e_emlrt_marshallIn(emlrtAlias(prhs[6]), "K", (real_T **)&K_data, K_size);

  /* Invoke the target function */
  onlineGPR(*x, *y, *x_star, *gram_matrix_data, gram_matrix_size, *X_data,
            X_size, *Y_data, Y_size, *K_data, K_size, *gram_matrix_update_data,
            gram_matrix_update_size, *y_star, &cov_star, *X_update_data,
            X_update_size, *Y_update_data, Y_update_size, *K_update_data,
            K_update_size);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*gram_matrix_update_data, gram_matrix_update_size);
  plhs[1] = b_emlrt_marshallOut(*y_star);
  plhs[2] = c_emlrt_marshallOut(cov_star);
  plhs[3] = d_emlrt_marshallOut(*X_update_data, X_update_size);
  plhs[4] = e_emlrt_marshallOut(*Y_update_data, Y_update_size);
  plhs[5] = emlrt_marshallOut(*K_update_data, K_update_size);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void onlineGPR_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  onlineGPR_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void onlineGPR_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, 0);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void onlineGPR_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_onlineGPR_api.c
 *
 * [EOF]
 */
