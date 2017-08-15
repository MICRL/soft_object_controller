//
// File: onlineGPR.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//
#ifndef ONLINEGPR_H
#define ONLINEGPR_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "onlineGPR_types.h"

// Function Declarations
extern void onlineGPR(const double x[6], const double y[6], const double x_star
                      [6], const double gram_matrix_data[], const int
                      gram_matrix_size[2], double X_data[], int X_size[2],
                      double Y_data[], int Y_size[2], double K_data[], int
                      K_size[2], double gram_matrix_update_data[], int
                      gram_matrix_update_size[2], double y_star[6], double
                      *cov_star, double X_update_data[], int X_update_size[2],
                      double Y_update_data[], int Y_update_size[2], double
                      K_update_data[], int K_update_size[2]);

#endif

//
// File trailer for onlineGPR.h
//
// [EOF]
//
