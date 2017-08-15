//
// File: inv.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//

// Include Files
#include "rt_nonfinite.h"
#include "onlineGPR.h"
#include "inv.h"

// Function Definitions

//
// Arguments    : const double x[4]
//                double y[4]
// Return Type  : void
//
void inv(const double x[4], double y[4])
{
  double r;
  double t;
  if (std::abs(x[1]) > std::abs(x[0])) {
    r = x[0] / x[1];
    t = 1.0 / (r * x[3] - x[2]);
    y[0] = x[3] / x[1] * t;
    y[1] = -t;
    y[2] = -x[2] / x[1] * t;
    y[3] = r * t;
  } else {
    r = x[1] / x[0];
    t = 1.0 / (x[3] - r * x[2]);
    y[0] = x[3] / x[0] * t;
    y[1] = -r * t;
    y[2] = -x[2] / x[0] * t;
    y[3] = t;
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
