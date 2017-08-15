//
// File: mean.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//

// Include Files
#include "rt_nonfinite.h"
#include "onlineGPR.h"
#include "mean.h"

// Function Definitions

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
void mean(const double x_data[], const int x_size[2], double y_data[], int
          y_size[2])
{
  int vlen;
  int i;
  int xoffset;
  double s;
  int k;
  y_size[1] = (signed char)x_size[1];
  if ((x_size[0] == 0) || (x_size[1] == 0)) {
    vlen = (signed char)x_size[1];
    for (i = 0; i < vlen; i++) {
      y_data[i] = 0.0;
    }
  } else {
    vlen = x_size[0];
    for (i = 0; i + 1 <= x_size[1]; i++) {
      xoffset = i * vlen;
      s = x_data[xoffset];
      for (k = 2; k <= vlen; k++) {
        s += x_data[(xoffset + k) - 1];
      }

      y_data[i] = s;
    }
  }

  y_size[0] = 1;
  vlen = (signed char)x_size[1];
  for (i = 0; i < vlen; i++) {
    y_data[i] /= (double)x_size[0];
  }
}

//
// File trailer for mean.cpp
//
// [EOF]
//
