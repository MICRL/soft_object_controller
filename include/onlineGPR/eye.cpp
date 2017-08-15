//
// File: eye.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//

// Include Files
#include "rt_nonfinite.h"
#include "onlineGPR.h"
#include "eye.h"

// Function Definitions

//
// Arguments    : double I[4]
// Return Type  : void
//
void eye(double I[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    I[k] = 0.0;
  }

  for (k = 0; k < 2; k++) {
    I[k + (k << 1)] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
