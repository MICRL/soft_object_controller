//
// File: inverseUpdate.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//

// Include Files
#include "rt_nonfinite.h"
#include "onlineGPR.h"
#include "inverseUpdate.h"

// Function Definitions

//
// INVERSE UPDATE USING BLOCK INVERSE LEMMA
// inv_A_update = inverseUpdate(inv_A,b,c) computes the inverse of a matrix obtained
// formula : inv([A b;b' c]).  b is column vectors and c is a real number.
// input  : The Inverse of Matrix A and vectors b and real number c  -->  (inv_A,b,c)
// output : The Inverse Matrix of [A b;b' c]  -->  inv_A_update
// Arguments    : const double inv_A_data[]
//                const int inv_A_size[2]
//                const double b_data[]
//                const int b_size[1]
//                double inv_A_update_data[]
//                int inv_A_update_size[2]
// Return Type  : void
//
void inverseUpdate(const double inv_A_data[], const int inv_A_size[2], const
                   double b_data[], const int b_size[1], double
                   inv_A_update_data[], int inv_A_update_size[2])
{
  int loop_ub;
  int i3;
  double a_data[101];
  int k;
  int y_size_idx_1;
  int n;
  double y_data[101];
  int m;
  double i_k;
  int i4;
  int br;
  int ic;
  int ar;
  double b_y_data[10000];
  int ib;
  double c_y_data[100];
  int ia;
  double C_data[10000];
  int u0;
  boolean_T empty_non_axis_sizes;
  int result_size_idx_1;
  static double result_data[10100];
  double d_y_data[100];
  loop_ub = b_size[0];
  for (i3 = 0; i3 < loop_ub; i3++) {
    a_data[i3] = b_data[i3];
  }

  if ((b_size[0] == 1) || (inv_A_size[0] == 1)) {
    y_size_idx_1 = inv_A_size[1];
    loop_ub = inv_A_size[1];
    for (i3 = 0; i3 < loop_ub; i3++) {
      y_data[i3] = 0.0;
      m = b_size[0];
      for (i4 = 0; i4 < m; i4++) {
        y_data[i3] += a_data[i4] * inv_A_data[i4 + inv_A_size[0] * i3];
      }
    }
  } else {
    k = b_size[0];
    y_size_idx_1 = (signed char)inv_A_size[1];
    n = inv_A_size[1] - 1;
    loop_ub = (signed char)inv_A_size[1];
    for (i3 = 0; i3 < loop_ub; i3++) {
      y_data[i3] = 0.0;
    }

    if (inv_A_size[1] != 0) {
      for (loop_ub = 1; loop_ub - 1 <= n; loop_ub++) {
        for (ic = loop_ub; ic <= loop_ub; ic++) {
          y_data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (loop_ub = 0; loop_ub <= n; loop_ub++) {
        ar = 0;
        i3 = br + k;
        for (ib = br; ib + 1 <= i3; ib++) {
          if (inv_A_data[ib] != 0.0) {
            ia = ar;
            for (ic = loop_ub; ic + 1 <= loop_ub + 1; ic++) {
              ia++;
              y_data[ic] += inv_A_data[ib] * a_data[ia - 1];
            }
          }

          ar++;
        }

        br += k;
      }
    }
  }

  if ((y_size_idx_1 == 1) || (b_size[0] == 1)) {
    i_k = 0.0;
    for (i3 = 0; i3 < y_size_idx_1; i3++) {
      i_k += y_data[i3] * b_data[i3];
    }
  } else {
    i_k = 0.0;
    for (i3 = 0; i3 < y_size_idx_1; i3++) {
      i_k += y_data[i3] * b_data[i3];
    }
  }

  i_k = 1.0 / (1.001 - i_k);
  loop_ub = inv_A_size[0] * inv_A_size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    b_y_data[i3] = i_k * inv_A_data[i3];
  }

  if ((inv_A_size[1] == 1) || (b_size[0] == 1)) {
    y_size_idx_1 = inv_A_size[0];
    loop_ub = inv_A_size[0];
    for (i3 = 0; i3 < loop_ub; i3++) {
      c_y_data[i3] = 0.0;
      m = inv_A_size[1];
      for (i4 = 0; i4 < m; i4++) {
        c_y_data[i3] += b_y_data[i3 + inv_A_size[0] * i4] * b_data[i4];
      }
    }
  } else {
    k = inv_A_size[1];
    m = inv_A_size[0];
    n = (signed char)inv_A_size[0];
    y_size_idx_1 = (signed char)inv_A_size[0];
    for (i3 = 0; i3 < n; i3++) {
      c_y_data[i3] = 0.0;
    }

    if (inv_A_size[0] != 0) {
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= 0)) {
        for (ic = 1; ic <= m; ic++) {
          c_y_data[ic - 1] = 0.0;
        }

        loop_ub = m;
      }

      br = 0;
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= 0)) {
        ar = 0;
        i3 = br + k;
        for (ib = br; ib + 1 <= i3; ib++) {
          if (b_data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= m; ic++) {
              ia++;
              c_y_data[ic] += b_data[ib] * b_y_data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        loop_ub = m;
      }
    }
  }

  for (i3 = 0; i3 < y_size_idx_1; i3++) {
    loop_ub = b_size[0];
    for (i4 = 0; i4 < loop_ub; i4++) {
      b_y_data[i3 + y_size_idx_1 * i4] = c_y_data[i3] * b_data[i4];
    }
  }

  if ((b_size[0] == 1) || (inv_A_size[0] == 1)) {
    for (i3 = 0; i3 < y_size_idx_1; i3++) {
      loop_ub = inv_A_size[1];
      for (i4 = 0; i4 < loop_ub; i4++) {
        C_data[i3 + y_size_idx_1 * i4] = 0.0;
        m = b_size[0];
        for (n = 0; n < m; n++) {
          C_data[i3 + y_size_idx_1 * i4] += b_y_data[i3 + y_size_idx_1 * n] *
            inv_A_data[n + inv_A_size[0] * i4];
        }
      }
    }
  } else {
    k = b_size[0];
    n = (signed char)y_size_idx_1;
    loop_ub = (signed char)inv_A_size[1];
    for (i3 = 0; i3 < loop_ub; i3++) {
      for (i4 = 0; i4 < n; i4++) {
        C_data[i4 + (signed char)y_size_idx_1 * i3] = 0.0;
      }
    }

    if ((y_size_idx_1 == 0) || (inv_A_size[1] == 0)) {
    } else {
      n = y_size_idx_1 * (inv_A_size[1] - 1);
      loop_ub = 0;
      while ((y_size_idx_1 > 0) && (loop_ub <= n)) {
        i3 = loop_ub + y_size_idx_1;
        for (ic = loop_ub; ic + 1 <= i3; ic++) {
          C_data[ic] = 0.0;
        }

        loop_ub += y_size_idx_1;
      }

      br = 0;
      loop_ub = 0;
      while ((y_size_idx_1 > 0) && (loop_ub <= n)) {
        ar = 0;
        i3 = br + k;
        for (ib = br; ib + 1 <= i3; ib++) {
          if (inv_A_data[ib] != 0.0) {
            ia = ar;
            i4 = loop_ub + y_size_idx_1;
            for (ic = loop_ub; ic + 1 <= i4; ic++) {
              ia++;
              C_data[ic] += inv_A_data[ib] * b_y_data[ia - 1];
            }
          }

          ar += y_size_idx_1;
        }

        br += k;
        loop_ub += y_size_idx_1;
      }
    }
  }

  loop_ub = inv_A_size[0] * inv_A_size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    b_y_data[i3] = -i_k * inv_A_data[i3];
  }

  if ((inv_A_size[1] == 1) || (b_size[0] == 1)) {
    y_size_idx_1 = inv_A_size[0];
    loop_ub = inv_A_size[0];
    for (i3 = 0; i3 < loop_ub; i3++) {
      c_y_data[i3] = 0.0;
      m = inv_A_size[1];
      for (i4 = 0; i4 < m; i4++) {
        c_y_data[i3] += b_y_data[i3 + inv_A_size[0] * i4] * b_data[i4];
      }
    }
  } else {
    k = inv_A_size[1];
    m = inv_A_size[0];
    n = (signed char)inv_A_size[0];
    y_size_idx_1 = (signed char)inv_A_size[0];
    for (i3 = 0; i3 < n; i3++) {
      c_y_data[i3] = 0.0;
    }

    if (inv_A_size[0] != 0) {
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= 0)) {
        for (ic = 1; ic <= m; ic++) {
          c_y_data[ic - 1] = 0.0;
        }

        loop_ub = m;
      }

      br = 0;
      loop_ub = 0;
      while ((m > 0) && (loop_ub <= 0)) {
        ar = 0;
        i3 = br + k;
        for (ib = br; ib + 1 <= i3; ib++) {
          if (b_data[ib] != 0.0) {
            ia = ar;
            for (ic = 0; ic + 1 <= m; ic++) {
              ia++;
              c_y_data[ic] += b_data[ib] * b_y_data[ia - 1];
            }
          }

          ar += m;
        }

        br += k;
        loop_ub = m;
      }
    }
  }

  loop_ub = inv_A_size[0] * inv_A_size[1];
  for (i3 = 0; i3 < loop_ub; i3++) {
    C_data[i3] += inv_A_data[i3];
  }

  if (!((inv_A_size[0] == 0) || (inv_A_size[1] == 0))) {
    u0 = inv_A_size[0];
  } else if (!(y_size_idx_1 == 0)) {
    u0 = y_size_idx_1;
  } else {
    u0 = inv_A_size[0];
    if (!(u0 > 0)) {
      u0 = 0;
    }
  }

  empty_non_axis_sizes = (u0 == 0);
  if (empty_non_axis_sizes || (!((inv_A_size[0] == 0) || (inv_A_size[1] == 0))))
  {
    m = inv_A_size[1];
  } else {
    m = 0;
  }

  if (empty_non_axis_sizes || (!(y_size_idx_1 == 0))) {
    n = 1;
  } else {
    n = 0;
  }

  result_size_idx_1 = m + n;
  for (i3 = 0; i3 < m; i3++) {
    for (i4 = 0; i4 < u0; i4++) {
      result_data[i4 + u0 * i3] = C_data[i4 + u0 * i3];
    }
  }

  for (i3 = 0; i3 < n; i3++) {
    for (i4 = 0; i4 < u0; i4++) {
      result_data[i4 + u0 * (i3 + m)] = c_y_data[i4 + u0 * i3];
    }
  }

  loop_ub = b_size[0];
  for (i3 = 0; i3 < loop_ub; i3++) {
    d_y_data[i3] = -i_k * b_data[i3];
  }

  if ((b_size[0] == 1) || (inv_A_size[0] == 1)) {
    y_size_idx_1 = inv_A_size[1];
    loop_ub = inv_A_size[1];
    for (i3 = 0; i3 < loop_ub; i3++) {
      y_data[i3] = 0.0;
      m = b_size[0];
      for (i4 = 0; i4 < m; i4++) {
        y_data[i3] += d_y_data[i4] * inv_A_data[i4 + inv_A_size[0] * i3];
      }
    }
  } else {
    k = b_size[0];
    y_size_idx_1 = (signed char)inv_A_size[1];
    n = inv_A_size[1] - 1;
    loop_ub = (signed char)inv_A_size[1];
    for (i3 = 0; i3 < loop_ub; i3++) {
      y_data[i3] = 0.0;
    }

    if (inv_A_size[1] != 0) {
      for (loop_ub = 1; loop_ub - 1 <= n; loop_ub++) {
        for (ic = loop_ub; ic <= loop_ub; ic++) {
          y_data[ic - 1] = 0.0;
        }
      }

      br = 0;
      for (loop_ub = 0; loop_ub <= n; loop_ub++) {
        ar = 0;
        i3 = br + k;
        for (ib = br; ib + 1 <= i3; ib++) {
          if (inv_A_data[ib] != 0.0) {
            ia = ar;
            for (ic = loop_ub; ic + 1 <= loop_ub + 1; ic++) {
              ia++;
              y_data[ic] += inv_A_data[ib] * d_y_data[ia - 1];
            }
          }

          ar++;
        }

        br += k;
      }
    }
  }

  for (i3 = 0; i3 < y_size_idx_1; i3++) {
    a_data[i3] = y_data[i3];
  }

  a_data[y_size_idx_1] = i_k;
  if (!((u0 == 0) || (result_size_idx_1 == 0))) {
    m = result_size_idx_1;
    n = u0;
  } else {
    m = y_size_idx_1 + 1;
    n = 0;
  }

  inv_A_update_size[0] = n + 1;
  inv_A_update_size[1] = m;
  for (i3 = 0; i3 < m; i3++) {
    for (i4 = 0; i4 < n; i4++) {
      inv_A_update_data[i4 + (n + 1) * i3] = result_data[i4 + n * i3];
    }
  }

  for (i3 = 0; i3 < m; i3++) {
    for (i4 = 0; i4 < 1; i4++) {
      inv_A_update_data[n + (n + 1) * i3] = a_data[i3];
    }
  }
}

//
// File trailer for inverseUpdate.cpp
//
// [EOF]
//
