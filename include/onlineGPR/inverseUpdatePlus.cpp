//
// File: inverseUpdatePlus.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//

// Include Files
#include "rt_nonfinite.h"
#include "onlineGPR.h"
#include "inverseUpdatePlus.h"
#include "inv.h"
#include "eye.h"

// Function Definitions

//
// INVERSEUPDATEPLUS Summary of this function goes here
//    Detailed explanation goes here
// Arguments    : const double inv_A_data[]
//                const int inv_A_size[2]
//                const double u[200]
//                const double v[200]
//                double inv_A_update_data[]
//                int inv_A_update_size[2]
// Return Type  : void
//
void inverseUpdatePlus(const double inv_A_data[], const int inv_A_size[2], const
  double u[200], const double v[200], double inv_A_update_data[], int
  inv_A_update_size[2])
{
  int i1;
  int i2;
  int y_size_idx_1;
  int loop_ub;
  double a[200];
  int m;
  double y_data[200];
  int k;
  double alpha[4];
  int br;
  int ic;
  double dv0[4];
  double dv1[4];
  int ar;
  int ib;
  int ia;
  double b_y_data[200];
  double c_y_data[10000];
  for (i1 = 0; i1 < 100; i1++) {
    for (i2 = 0; i2 < 2; i2++) {
      a[i2 + (i1 << 1)] = v[i1 + 100 * i2];
    }
  }

  if (inv_A_size[0] == 1) {
    y_size_idx_1 = inv_A_size[1];
    loop_ub = inv_A_size[1];
    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < loop_ub; i2++) {
        y_data[i1 + (i2 << 1)] = 0.0;
        for (k = 0; k < 100; k++) {
          y_data[i1 + (i2 << 1)] += a[i1 + (k << 1)] * inv_A_data[k + i2];
        }
      }
    }
  } else {
    y_size_idx_1 = (signed char)inv_A_size[1];
    loop_ub = (signed char)inv_A_size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        y_data[i2 + (i1 << 1)] = 0.0;
      }
    }

    if (inv_A_size[1] != 0) {
      m = (inv_A_size[1] - 1) << 1;
      for (loop_ub = 0; loop_ub <= m; loop_ub += 2) {
        for (ic = loop_ub + 1; ic <= loop_ub + 2; ic++) {
          y_data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (loop_ub = 0; loop_ub <= m; loop_ub += 2) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 100; ib++) {
          if (inv_A_data[ib] != 0.0) {
            ia = ar;
            for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
              ia++;
              y_data[ic] += inv_A_data[ib] * a[ia];
            }
          }

          ar += 2;
        }

        br += 100;
      }
    }
  }

  if (y_size_idx_1 == 1) {
    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        alpha[i1 + (i2 << 1)] = 0.0;
        for (k = 0; k < 100; k++) {
          alpha[i1 + (i2 << 1)] += y_data[i1 + 2 * k] * u[k + 100 * i2];
        }
      }
    }
  } else {
    for (i1 = 0; i1 < 4; i1++) {
      alpha[i1] = 0.0;
    }

    for (loop_ub = 0; loop_ub <= 3; loop_ub += 2) {
      for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
        alpha[ic] = 0.0;
      }
    }

    br = 0;
    for (loop_ub = 0; loop_ub <= 3; loop_ub += 2) {
      ar = -1;
      i1 = br + y_size_idx_1;
      for (ib = br; ib + 1 <= i1; ib++) {
        if (u[ib] != 0.0) {
          ia = ar;
          for (ic = loop_ub; ic + 1 <= loop_ub + 2; ic++) {
            ia++;
            alpha[ic] += u[ib] * y_data[ia];
          }
        }

        ar += 2;
      }

      br += y_size_idx_1;
    }
  }

  eye(dv0);
  for (i1 = 0; i1 < 4; i1++) {
    dv1[i1] = dv0[i1] + alpha[i1];
  }

  inv(dv1, alpha);
  if (inv_A_size[1] == 1) {
    y_size_idx_1 = inv_A_size[0];
    loop_ub = inv_A_size[0];
    for (i1 = 0; i1 < loop_ub; i1++) {
      for (i2 = 0; i2 < 2; i2++) {
        y_data[i1 + y_size_idx_1 * i2] = 0.0;
        for (k = 0; k < 1; k++) {
          y_data[i1 + y_size_idx_1 * i2] += inv_A_data[i1] * u[100 * i2];
        }
      }
    }
  } else {
    k = inv_A_size[1];
    y_size_idx_1 = (signed char)inv_A_size[0];
    m = inv_A_size[0];
    loop_ub = (signed char)inv_A_size[0];
    for (i1 = 0; i1 < 2; i1++) {
      for (i2 = 0; i2 < loop_ub; i2++) {
        y_data[i2 + y_size_idx_1 * i1] = 0.0;
      }
    }

    if (inv_A_size[0] != 0) {
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= m)) {
        i1 = loop_ub + m;
        for (ic = loop_ub; ic + 1 <= i1; ic++) {
          y_data[ic] = 0.0;
        }

        loop_ub += m;
      }

      br = 0;
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= m)) {
        ar = -1;
        i1 = br + k;
        for (ib = br; ib + 1 <= i1; ib++) {
          if (u[ib] != 0.0) {
            ia = ar;
            i2 = loop_ub + m;
            for (ic = loop_ub; ic + 1 <= i2; ic++) {
              ia++;
              y_data[ic] += u[ib] * inv_A_data[ia];
            }
          }

          ar += m;
        }

        br += k;
        loop_ub += m;
      }
    }
  }

  loop_ub = (signed char)y_size_idx_1;
  for (i1 = 0; i1 < 2; i1++) {
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_y_data[i2 + (signed char)y_size_idx_1 * i1] = 0.0;
    }
  }

  if (y_size_idx_1 != 0) {
    loop_ub = 0;
    while ((y_size_idx_1 > 0) && (loop_ub <= y_size_idx_1)) {
      i1 = loop_ub + y_size_idx_1;
      for (ic = loop_ub; ic + 1 <= i1; ic++) {
        b_y_data[ic] = 0.0;
      }

      loop_ub += y_size_idx_1;
    }

    br = 0;
    loop_ub = 0;
    while ((y_size_idx_1 > 0) && (loop_ub <= y_size_idx_1)) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (alpha[ib] != 0.0) {
          ia = ar;
          i1 = loop_ub + y_size_idx_1;
          for (ic = loop_ub; ic + 1 <= i1; ic++) {
            ia++;
            b_y_data[ic] += alpha[ib] * y_data[ia];
          }
        }

        ar += y_size_idx_1;
      }

      br += 2;
      loop_ub += y_size_idx_1;
    }
  }

  loop_ub = (signed char)y_size_idx_1;
  for (i1 = 0; i1 < 100; i1++) {
    for (i2 = 0; i2 < 2; i2++) {
      a[i2 + (i1 << 1)] = v[i1 + 100 * i2];
    }

    for (i2 = 0; i2 < loop_ub; i2++) {
      c_y_data[i2 + (signed char)y_size_idx_1 * i1] = 0.0;
    }
  }

  if ((signed char)y_size_idx_1 != 0) {
    m = (signed char)y_size_idx_1 * 99;
    loop_ub = 0;
    while (((signed char)y_size_idx_1 > 0) && (loop_ub <= m)) {
      i1 = loop_ub + (signed char)y_size_idx_1;
      for (ic = loop_ub; ic + 1 <= i1; ic++) {
        c_y_data[ic] = 0.0;
      }

      loop_ub += (signed char)y_size_idx_1;
    }

    br = 0;
    loop_ub = 0;
    while (((signed char)y_size_idx_1 > 0) && (loop_ub <= m)) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 2; ib++) {
        if (a[ib] != 0.0) {
          ia = ar;
          i1 = loop_ub + (signed char)y_size_idx_1;
          for (ic = loop_ub; ic + 1 <= i1; ic++) {
            ia++;
            c_y_data[ic] += a[ib] * b_y_data[ia];
          }
        }

        ar += (signed char)y_size_idx_1;
      }

      br += 2;
      loop_ub += (signed char)y_size_idx_1;
    }
  }

  if (inv_A_size[0] == 1) {
    loop_ub = (signed char)y_size_idx_1;
    for (i1 = 0; i1 < loop_ub; i1++) {
      m = inv_A_size[1];
      for (i2 = 0; i2 < m; i2++) {
        inv_A_update_data[i1 + (signed char)y_size_idx_1 * i2] = 0.0;
        for (k = 0; k < 100; k++) {
          inv_A_update_data[i1 + (signed char)y_size_idx_1 * i2] += c_y_data[i1
            + (signed char)y_size_idx_1 * k] * inv_A_data[k + i2];
        }
      }
    }
  } else {
    loop_ub = (signed char)inv_A_size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      m = (signed char)y_size_idx_1;
      for (i2 = 0; i2 < m; i2++) {
        inv_A_update_data[i2 + (signed char)y_size_idx_1 * i1] = 0.0;
      }
    }

    if (((signed char)y_size_idx_1 == 0) || (inv_A_size[1] == 0)) {
    } else {
      m = (signed char)y_size_idx_1 * (inv_A_size[1] - 1);
      loop_ub = 0;
      while (((signed char)y_size_idx_1 > 0) && (loop_ub <= m)) {
        i1 = loop_ub + (signed char)y_size_idx_1;
        for (ic = loop_ub; ic + 1 <= i1; ic++) {
          inv_A_update_data[ic] = 0.0;
        }

        loop_ub += (signed char)y_size_idx_1;
      }

      br = 0;
      loop_ub = 0;
      while (((signed char)y_size_idx_1 > 0) && (loop_ub <= m)) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 100; ib++) {
          if (inv_A_data[ib] != 0.0) {
            ia = ar;
            i1 = loop_ub + (signed char)y_size_idx_1;
            for (ic = loop_ub; ic + 1 <= i1; ic++) {
              ia++;
              inv_A_update_data[ic] += inv_A_data[ib] * c_y_data[ia];
            }
          }

          ar += (signed char)y_size_idx_1;
        }

        br += 100;
        loop_ub += (signed char)y_size_idx_1;
      }
    }
  }

  inv_A_update_size[0] = inv_A_size[0];
  inv_A_update_size[1] = inv_A_size[1];
  loop_ub = inv_A_size[0] * inv_A_size[1];
  for (i1 = 0; i1 < loop_ub; i1++) {
    inv_A_update_data[i1] = inv_A_data[i1] - inv_A_update_data[i1];
  }
}

//
// File trailer for inverseUpdatePlus.cpp
//
// [EOF]
//
