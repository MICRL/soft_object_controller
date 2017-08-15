//
// File: onlineGPR.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//

// Include Files
#include "rt_nonfinite.h"
#include "onlineGPR.h"
#include "inverseUpdatePlus.h"
#include "mean.h"
#include "inverseUpdate.h"

// Type Definitions
#ifndef struct_emxArray_real_T_100x101
#define struct_emxArray_real_T_100x101

struct emxArray_real_T_100x101
{
  double data[10100];
  int size[2];
};

#endif                                 //struct_emxArray_real_T_100x101

#ifndef struct_s3iKhQYLKYEx0HggWdB9C5F
#define struct_s3iKhQYLKYEx0HggWdB9C5F

struct s3iKhQYLKYEx0HggWdB9C5F
{
  emxArray_real_T_100x101 f1;
};

#endif                                 //struct_s3iKhQYLKYEx0HggWdB9C5F

typedef s3iKhQYLKYEx0HggWdB9C5F cell_wrap_1;

// Function Definitions

//
// ONLINEGPR Summary of this function goes here
//    Detailed explanation goes here
//  The max size of gram matrix
// Arguments    : const double x[6]
//                const double y[6]
//                const double x_star[6]
//                const double gram_matrix_data[]
//                const int gram_matrix_size[2]
//                double X_data[]
//                int X_size[2]
//                double Y_data[]
//                int Y_size[2]
//                double K_data[]
//                int K_size[2]
//                double gram_matrix_update_data[]
//                int gram_matrix_update_size[2]
//                double y_star[6]
//                double *cov_star
//                double X_update_data[]
//                int X_update_size[2]
//                double Y_update_data[]
//                int Y_update_size[2]
//                double K_update_data[]
//                int K_update_size[2]
// Return Type  : void
//
void onlineGPR(const double x[6], const double y[6], const double x_star[6],
               const double gram_matrix_data[], const int gram_matrix_size[2],
               double X_data[], int X_size[2], double Y_data[], int Y_size[2],
               double K_data[], int K_size[2], double gram_matrix_update_data[],
               int gram_matrix_update_size[2], double y_star[6], double
               *cov_star, double X_update_data[], int X_update_size[2], double
               Y_update_data[], int Y_update_size[2], double K_update_data[],
               int K_update_size[2])
{
  int ar;
  int ixstart;
  double varargin_1_data[100];
  int varargin_1_size[2];
  int n;
  double mtmp;
  int br;
  int i0;
  boolean_T exitg1;
  int loop_ub;
  int C_size[1];
  double C_data[101];
  int b_loop_ub;
  double a_data[101];
  int k;
  double b_a_data[101];
  int ia;
  boolean_T empty_non_axis_sizes;
  double Ks_old_data[100];
  static cell_wrap_1 reshapes[2];
  static double result_data[10100];
  double varargin_2_data[101];
  double u[200];
  double v[200];
  static double tmp_data[10000];
  double c_a_data[101];
  int varargin_2_size_idx_1;

  //  Kernel function
  if ((gram_matrix_size[0] == 0) || (gram_matrix_size[1] == 0)) {
    ixstart = 0;
  } else {
    ar = gram_matrix_size[0];
    ixstart = gram_matrix_size[1];
    if (ar > ixstart) {
      ixstart = ar;
    }
  }

  //      % If it is the first time we compute gram marix and return
  //      if(gram_size == 0)
  //          X_update = x;
  //          Y_update = y;
  //          K_update = 1;
  //          gram_matrix_update = inv(K_update+keps*eye(length(K_update)));
  //  When the size of gram matrix reached a certain number,we use
  //  Sherman-Morrison formula to update the matrix which means we choose
  //  one row and column to be replaced by new data
  if (ixstart == 100) {
    //  choose one row and column which has the highest covariance
    mean(K_data, K_size, varargin_1_data, varargin_1_size);
    ixstart = 1;
    n = varargin_1_size[1];
    mtmp = varargin_1_data[0];
    ar = 0;
    if (varargin_1_size[1] > 1) {
      if (rtIsNaN(varargin_1_data[0])) {
        br = 1;
        exitg1 = false;
        while ((!exitg1) && (br + 1 <= n)) {
          ixstart = br + 1;
          if (!rtIsNaN(varargin_1_data[br])) {
            mtmp = varargin_1_data[br];
            ar = br;
            exitg1 = true;
          } else {
            br++;
          }
        }
      }

      if (ixstart < varargin_1_size[1]) {
        while (ixstart + 1 <= n) {
          if (varargin_1_data[ixstart] > mtmp) {
            mtmp = varargin_1_data[ixstart];
            ar = ixstart;
          }

          ixstart++;
        }
      }
    }

    //  K(:,index);
    for (i0 = 0; i0 < 6; i0++) {
      X_data[i0 + X_size[0] * ar] = x[i0];
    }

    X_update_size[0] = 6;
    X_update_size[1] = X_size[1];
    loop_ub = X_size[0] * X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      X_update_data[i0] = X_data[i0];
    }

    for (i0 = 0; i0 < 6; i0++) {
      Y_data[ar + Y_size[0] * i0] = y[i0];
    }

    Y_update_size[0] = Y_size[0];
    Y_update_size[1] = 6;
    loop_ub = Y_size[0] * Y_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      Y_update_data[i0] = Y_data[i0];
    }

    //  sq_dist - a function to compute a matrix of all pairwise squared distances 
    //  between two sets of vectors, stored in the columns of the two matrices, a 
    //  (of size D by n) and b (of size D by m). If only a single argument is given 
    //  or the second matrix is empty, the missing matrix is taken to be identical 
    //  to the first.
    //
    //  Special functionality: If an optional third matrix argument Q is given, it 
    //  must be of size n by m, and in this case a vector of the traces of the
    //  product of Q' and the coordinatewise squared distances is returned.
    //
    //  NOTE: The program code is written in the C language for efficiency and is 
    //  contained in the file sq_dist.c, and should be compiled using matlabs mex 
    //  facility. However, this file also contains a (less efficient) matlab
    //  implementation, supplied only as a help to people unfamiliar with mex. If 
    //  the C code has been properly compiled and is avaiable, it automatically
    //  takes precendence over the matlab code in this file.
    //
    //  Usage: C = sq_dist(a, b)
    //     or: C = sq_dist(a)  or equiv.: C = sq_dist(a, [])
    //     or: c = sq_dist(a, b, Q)
    //  where the b matrix may be empty.
    //
    //  where a is of size D by n, b is of size D by m (or empty), C and Q are of 
    //  size n by m and c is of size D by 1.
    //
    //  Copyright (c) 2003, 2004, 2005 and 2006 Carl Edward Rasmussen. 2006-03-09. 
    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      C_data[i0] = 0.0;
    }

    loop_ub = X_size[1];
    b_loop_ub = (signed char)X_size[1];
    ixstart = X_size[1];
    for (br = 0; br < 6; br++) {
      for (i0 = 0; i0 < loop_ub; i0++) {
        a_data[i0] = X_data[br + X_size[0] * i0];
      }

      if ((!(loop_ub == 0)) && (!((signed char)loop_ub == 0))) {
        for (k = 0; k + 1 <= loop_ub; k++) {
          b_a_data[k] = a_data[k];
        }
      }

      for (i0 = 0; i0 < b_loop_ub; i0++) {
        b_a_data[i0] = x[br] - b_a_data[i0];
      }

      for (k = 0; k + 1 <= (signed char)X_size[1]; k++) {
        a_data[k] = b_a_data[k] * b_a_data[k];
      }

      for (i0 = 0; i0 < ixstart; i0++) {
        C_data[i0] += a_data[i0];
      }
    }

    //  C = repmat(sum(a.*a)',1,m)+repmat(sum(b.*b),n,1)-2*a'*b could be used to  
    //  replace the 3 lines above; it would be faster, but numerically less stable. 
    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      C_data[i0] = -0.5 * C_data[i0] / 0.36;
    }

    for (k = 0; k + 1 <= X_size[1]; k++) {
      C_data[k] = std::exp(C_data[k]);
    }

    loop_ub = K_size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      Ks_old_data[i0] = K_data[i0 + K_size[0] * ar];
    }

    //          K_old = K;
    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      K_data[i0 + K_size[0] * ar] = C_data[i0];
    }

    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      K_data[ar + K_size[0] * i0] = C_data[i0];
    }

    K_update_size[0] = K_size[0];
    K_update_size[1] = K_size[1];
    loop_ub = K_size[0] * K_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      K_update_data[i0] = K_data[i0];
    }

    memset(&u[0], 0, 200U * sizeof(double));
    u[ar] = 1.0;
    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      varargin_1_data[i0] = C_data[i0] - Ks_old_data[i0];
    }

    memcpy(&u[100], &varargin_1_data[0], 100U * sizeof(double));
    u[100 + ar] /= 2.0;
    for (i0 = 0; i0 < 100; i0++) {
      v[i0] = u[100 + i0];
      v[100 + i0] = u[i0];
    }

    inverseUpdatePlus(gram_matrix_data, gram_matrix_size, u, v, tmp_data,
                      varargin_1_size);
    gram_matrix_update_size[0] = varargin_1_size[0];
    gram_matrix_update_size[1] = varargin_1_size[1];
    loop_ub = varargin_1_size[0] * varargin_1_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      gram_matrix_update_data[i0] = tmp_data[i0];
    }

    //  otherwise we use block matrix property to update the matrix
  } else {
    //  sq_dist - a function to compute a matrix of all pairwise squared distances 
    //  between two sets of vectors, stored in the columns of the two matrices, a 
    //  (of size D by n) and b (of size D by m). If only a single argument is given 
    //  or the second matrix is empty, the missing matrix is taken to be identical 
    //  to the first.
    //
    //  Special functionality: If an optional third matrix argument Q is given, it 
    //  must be of size n by m, and in this case a vector of the traces of the
    //  product of Q' and the coordinatewise squared distances is returned.
    //
    //  NOTE: The program code is written in the C language for efficiency and is 
    //  contained in the file sq_dist.c, and should be compiled using matlabs mex 
    //  facility. However, this file also contains a (less efficient) matlab
    //  implementation, supplied only as a help to people unfamiliar with mex. If 
    //  the C code has been properly compiled and is avaiable, it automatically
    //  takes precendence over the matlab code in this file.
    //
    //  Usage: C = sq_dist(a, b)
    //     or: C = sq_dist(a)  or equiv.: C = sq_dist(a, [])
    //     or: c = sq_dist(a, b, Q)
    //  where the b matrix may be empty.
    //
    //  where a is of size D by n, b is of size D by m (or empty), C and Q are of 
    //  size n by m and c is of size D by 1.
    //
    //  Copyright (c) 2003, 2004, 2005 and 2006 Carl Edward Rasmussen. 2006-03-09. 
    C_size[0] = X_size[1];
    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      C_data[i0] = 0.0;
    }

    loop_ub = X_size[1];
    b_loop_ub = (signed char)X_size[1];
    ixstart = X_size[1];
    for (br = 0; br < 6; br++) {
      for (i0 = 0; i0 < loop_ub; i0++) {
        a_data[i0] = X_data[br + X_size[0] * i0];
      }

      if ((!(loop_ub == 0)) && (!((signed char)loop_ub == 0))) {
        for (k = 0; k + 1 <= loop_ub; k++) {
          b_a_data[k] = a_data[k];
        }
      }

      for (i0 = 0; i0 < b_loop_ub; i0++) {
        b_a_data[i0] = x[br] - b_a_data[i0];
      }

      for (k = 0; k + 1 <= (signed char)X_size[1]; k++) {
        a_data[k] = b_a_data[k] * b_a_data[k];
      }

      for (i0 = 0; i0 < ixstart; i0++) {
        C_data[i0] += a_data[i0];
      }
    }

    //  C = repmat(sum(a.*a)',1,m)+repmat(sum(b.*b),n,1)-2*a'*b could be used to  
    //  replace the 3 lines above; it would be faster, but numerically less stable. 
    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      C_data[i0] = -0.5 * C_data[i0] / 0.36;
    }

    for (k = 0; k + 1 <= C_size[0]; k++) {
      C_data[k] = std::exp(C_data[k]);
    }

    inverseUpdate(gram_matrix_data, gram_matrix_size, C_data, C_size,
                  gram_matrix_update_data, gram_matrix_update_size);
    X_update_size[0] = 6;
    X_update_size[1] = X_size[1] + 1;
    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      for (ia = 0; ia < 6; ia++) {
        X_update_data[ia + 6 * i0] = X_data[ia + X_size[0] * i0];
      }
    }

    Y_update_size[0] = Y_size[0] + 1;
    Y_update_size[1] = 6;
    loop_ub = Y_size[0];
    for (i0 = 0; i0 < 6; i0++) {
      X_update_data[i0 + 6 * X_size[1]] = x[i0];
      for (ia = 0; ia < loop_ub; ia++) {
        Y_update_data[ia + Y_update_size[0] * i0] = Y_data[ia + Y_size[0] * i0];
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      Y_update_data[Y_size[0] + Y_update_size[0] * i0] = y[i0];
    }

    if (!((K_size[0] == 0) || (K_size[1] == 0))) {
      ar = K_size[0];
    } else if (!(X_size[1] == 0)) {
      ar = X_size[1];
    } else {
      ar = K_size[0];
      if (!(ar > 0)) {
        ar = 0;
      }
    }

    empty_non_axis_sizes = (ar == 0);
    if (empty_non_axis_sizes || (!((K_size[0] == 0) || (K_size[1] == 0)))) {
      ixstart = K_size[1];
    } else {
      ixstart = 0;
    }

    if (empty_non_axis_sizes || (!(X_size[1] == 0))) {
      br = 1;
    } else {
      br = 0;
    }

    loop_ub = ar * br;
    for (i0 = 0; i0 < loop_ub; i0++) {
      reshapes[1].f1.data[i0] = C_data[i0];
    }

    b_loop_ub = ixstart + br;
    for (i0 = 0; i0 < ixstart; i0++) {
      for (ia = 0; ia < ar; ia++) {
        result_data[ia + ar * i0] = K_data[ia + ar * i0];
      }
    }

    for (i0 = 0; i0 < br; i0++) {
      for (ia = 0; ia < ar; ia++) {
        result_data[ia + ar * (i0 + ixstart)] = reshapes[1].f1.data[ia + ar * i0];
      }
    }

    loop_ub = X_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      varargin_2_data[i0] = C_data[i0];
    }

    varargin_2_data[X_size[1]] = 1.0;
    if (!((ar == 0) || (b_loop_ub == 0))) {
      ixstart = b_loop_ub;
      br = ar;
    } else {
      ixstart = X_size[1] + 1;
      br = 0;
    }

    K_update_size[0] = br + 1;
    K_update_size[1] = ixstart;
    for (i0 = 0; i0 < ixstart; i0++) {
      for (ia = 0; ia < br; ia++) {
        K_update_data[ia + (br + 1) * i0] = result_data[ia + br * i0];
      }
    }

    for (i0 = 0; i0 < ixstart; i0++) {
      for (ia = 0; ia < 1; ia++) {
        K_update_data[br + (br + 1) * i0] = varargin_2_data[i0];
      }
    }
  }

  //  sq_dist - a function to compute a matrix of all pairwise squared distances 
  //  between two sets of vectors, stored in the columns of the two matrices, a
  //  (of size D by n) and b (of size D by m). If only a single argument is given 
  //  or the second matrix is empty, the missing matrix is taken to be identical 
  //  to the first.
  //
  //  Special functionality: If an optional third matrix argument Q is given, it 
  //  must be of size n by m, and in this case a vector of the traces of the
  //  product of Q' and the coordinatewise squared distances is returned.
  //
  //  NOTE: The program code is written in the C language for efficiency and is
  //  contained in the file sq_dist.c, and should be compiled using matlabs mex
  //  facility. However, this file also contains a (less efficient) matlab
  //  implementation, supplied only as a help to people unfamiliar with mex. If
  //  the C code has been properly compiled and is avaiable, it automatically
  //  takes precendence over the matlab code in this file.
  //
  //  Usage: C = sq_dist(a, b)
  //     or: C = sq_dist(a)  or equiv.: C = sq_dist(a, [])
  //     or: c = sq_dist(a, b, Q)
  //  where the b matrix may be empty.
  //
  //  where a is of size D by n, b is of size D by m (or empty), C and Q are of
  //  size n by m and c is of size D by 1.
  //
  //  Copyright (c) 2003, 2004, 2005 and 2006 Carl Edward Rasmussen. 2006-03-09. 
  loop_ub = X_update_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    C_data[i0] = 0.0;
  }

  loop_ub = X_update_size[1];
  b_loop_ub = (signed char)X_update_size[1];
  ixstart = X_update_size[1];
  for (br = 0; br < 6; br++) {
    for (i0 = 0; i0 < loop_ub; i0++) {
      a_data[i0] = X_update_data[br + 6 * i0];
    }

    if ((!(loop_ub == 0)) && (!((signed char)loop_ub == 0))) {
      for (k = 0; k + 1 <= loop_ub; k++) {
        b_a_data[k] = a_data[k];
      }
    }

    for (i0 = 0; i0 < b_loop_ub; i0++) {
      b_a_data[i0] = x_star[br] - b_a_data[i0];
    }

    for (k = 0; k + 1 <= (signed char)X_update_size[1]; k++) {
      a_data[k] = b_a_data[k] * b_a_data[k];
    }

    for (i0 = 0; i0 < ixstart; i0++) {
      C_data[i0] += a_data[i0];
    }
  }

  //  C = repmat(sum(a.*a)',1,m)+repmat(sum(b.*b),n,1)-2*a'*b could be used to
  //  replace the 3 lines above; it would be faster, but numerically less stable. 
  loop_ub = X_update_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    C_data[i0] = -0.5 * C_data[i0] / 0.36;
  }

  for (k = 0; k + 1 <= X_update_size[1]; k++) {
    C_data[k] = std::exp(C_data[k]);
  }

  loop_ub = X_update_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_a_data[i0] = C_data[i0];
  }

  if ((X_update_size[1] == 1) || (gram_matrix_update_size[0] == 1)) {
    varargin_2_size_idx_1 = gram_matrix_update_size[1];
    loop_ub = gram_matrix_update_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      varargin_2_data[i0] = 0.0;
      b_loop_ub = X_update_size[1];
      for (ia = 0; ia < b_loop_ub; ia++) {
        varargin_2_data[i0] += c_a_data[ia] * gram_matrix_update_data[ia +
          gram_matrix_update_size[0] * i0];
      }
    }
  } else {
    k = X_update_size[1];
    varargin_2_size_idx_1 = (signed char)gram_matrix_update_size[1];
    n = gram_matrix_update_size[1] - 1;
    loop_ub = (signed char)gram_matrix_update_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      varargin_2_data[i0] = 0.0;
    }

    if (gram_matrix_update_size[1] != 0) {
      for (ixstart = 1; ixstart - 1 <= n; ixstart++) {
        for (loop_ub = ixstart; loop_ub <= ixstart; loop_ub++) {
          varargin_2_data[loop_ub - 1] = 0.0;
        }
      }

      br = 0;
      for (ixstart = 0; ixstart <= n; ixstart++) {
        ar = 0;
        i0 = br + k;
        for (b_loop_ub = br; b_loop_ub + 1 <= i0; b_loop_ub++) {
          if (gram_matrix_update_data[b_loop_ub] != 0.0) {
            ia = ar;
            for (loop_ub = ixstart; loop_ub + 1 <= ixstart + 1; loop_ub++) {
              ia++;
              varargin_2_data[loop_ub] += gram_matrix_update_data[b_loop_ub] *
                c_a_data[ia - 1];
            }
          }

          ar++;
        }

        br += k;
      }
    }
  }

  if ((varargin_2_size_idx_1 == 1) || (Y_update_size[0] == 1)) {
    for (i0 = 0; i0 < 6; i0++) {
      y_star[i0] = 0.0;
      for (ia = 0; ia < varargin_2_size_idx_1; ia++) {
        y_star[i0] += varargin_2_data[ia] * Y_update_data[ia + Y_update_size[0] *
          i0];
      }
    }
  } else {
    for (i0 = 0; i0 < 6; i0++) {
      y_star[i0] = 0.0;
    }

    for (ixstart = 0; ixstart < 6; ixstart++) {
      for (loop_ub = ixstart; loop_ub + 1 <= ixstart + 1; loop_ub++) {
        y_star[loop_ub] = 0.0;
      }
    }

    br = 0;
    for (ixstart = 0; ixstart < 6; ixstart++) {
      ar = 0;
      i0 = br + varargin_2_size_idx_1;
      for (b_loop_ub = br; b_loop_ub + 1 <= i0; b_loop_ub++) {
        if (Y_update_data[b_loop_ub] != 0.0) {
          ia = ar;
          for (loop_ub = ixstart; loop_ub + 1 <= ixstart + 1; loop_ub++) {
            ia++;
            y_star[loop_ub] += Y_update_data[b_loop_ub] * varargin_2_data[ia - 1];
          }
        }

        ar++;
      }

      br += varargin_2_size_idx_1;
    }
  }

  loop_ub = X_update_size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_a_data[i0] = C_data[i0];
  }

  if ((X_update_size[1] == 1) || (gram_matrix_update_size[0] == 1)) {
    varargin_2_size_idx_1 = gram_matrix_update_size[1];
    loop_ub = gram_matrix_update_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      varargin_2_data[i0] = 0.0;
      b_loop_ub = X_update_size[1];
      for (ia = 0; ia < b_loop_ub; ia++) {
        varargin_2_data[i0] += c_a_data[ia] * gram_matrix_update_data[ia +
          gram_matrix_update_size[0] * i0];
      }
    }
  } else {
    k = X_update_size[1];
    varargin_2_size_idx_1 = (signed char)gram_matrix_update_size[1];
    n = gram_matrix_update_size[1] - 1;
    loop_ub = (signed char)gram_matrix_update_size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      varargin_2_data[i0] = 0.0;
    }

    if (gram_matrix_update_size[1] != 0) {
      for (ixstart = 1; ixstart - 1 <= n; ixstart++) {
        for (loop_ub = ixstart; loop_ub <= ixstart; loop_ub++) {
          varargin_2_data[loop_ub - 1] = 0.0;
        }
      }

      br = 0;
      for (ixstart = 0; ixstart <= n; ixstart++) {
        ar = 0;
        i0 = br + k;
        for (b_loop_ub = br; b_loop_ub + 1 <= i0; b_loop_ub++) {
          if (gram_matrix_update_data[b_loop_ub] != 0.0) {
            ia = ar;
            for (loop_ub = ixstart; loop_ub + 1 <= ixstart + 1; loop_ub++) {
              ia++;
              varargin_2_data[loop_ub] += gram_matrix_update_data[b_loop_ub] *
                c_a_data[ia - 1];
            }
          }

          ar++;
        }

        br += k;
      }
    }
  }

  if ((varargin_2_size_idx_1 == 1) || (X_update_size[1] == 1)) {
    mtmp = 0.0;
    for (i0 = 0; i0 < varargin_2_size_idx_1; i0++) {
      mtmp += varargin_2_data[i0] * C_data[i0];
    }
  } else {
    mtmp = 0.0;
    for (i0 = 0; i0 < varargin_2_size_idx_1; i0++) {
      mtmp += varargin_2_data[i0] * C_data[i0];
    }
  }

  *cov_star = 1.0 - mtmp;
}

//
// File trailer for onlineGPR.cpp
//
// [EOF]
//
