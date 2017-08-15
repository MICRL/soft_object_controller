//
// File: main.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 10-Aug-2017 21:27:39
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "onlineGPR.h"
#include "main.h"
#include "onlineGPR_terminate.h"
#include "onlineGPR_initialize.h"

// Function Declarations
static void argInit_1x6_real_T(double result[6]);
static void argInit_6x1_real_T(double result[6]);
static void argInit_6xd100_real_T(double result_data[], int result_size[2]);
static void argInit_d100x6_real_T(double result_data[], int result_size[2]);
static void argInit_d100xd100_real_T(double result_data[], int result_size[2]);
static double argInit_real_T();
static void main_onlineGPR();

// Function Definitions

//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_1x6_real_T(double result[6])
{
  int idx1;

  // Loop over the array to initialize each element.
  for (idx1 = 0; idx1 < 6; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_6x1_real_T(double result[6])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 6; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_6xd100_real_T(double result_data[], int result_size[2])
{
  int idx0;
  int idx1;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 6;
  result_size[1] = 2;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 6; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result_data[idx0 + 6 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_d100x6_real_T(double result_data[], int result_size[2])
{
  int idx0;
  int idx1;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 2;
  result_size[1] = 6;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 2; idx0++) {
    for (idx1 = 0; idx1 < 6; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result_data[idx0 + 2 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result_data[]
//                int result_size[2]
// Return Type  : void
//
static void argInit_d100xd100_real_T(double result_data[], int result_size[2])
{
  int idx0;
  int idx1;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 2;
  result_size[1] = 2;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 2; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result_data[idx0 + 2 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_onlineGPR()
{
  double x[6];
  double y[6];
  double x_star[6];
  static double gram_matrix_data[10000];
  int gram_matrix_size[2];
  double X_data[600];
  int X_size[2];
  double Y_data[600];
  int Y_size[2];
  static double K_data[10000];
  int K_size[2];
  static double gram_matrix_update_data[10201];
  int gram_matrix_update_size[2];
  double y_star[6];
  double cov_star;
  double X_update_data[606];
  int X_update_size[2];
  double Y_update_data[606];
  int Y_update_size[2];
  static double K_update_data[10201];
  int K_update_size[2];

  // Initialize function 'onlineGPR' input arguments.
  // Initialize function input argument 'x'.
  argInit_6x1_real_T(x);

  // Initialize function input argument 'y'.
  argInit_1x6_real_T(y);

  // Initialize function input argument 'x_star'.
  argInit_6x1_real_T(x_star);

  // Initialize function input argument 'gram_matrix'.
  argInit_d100xd100_real_T(gram_matrix_data, gram_matrix_size);

  // Initialize function input argument 'X'.
  argInit_6xd100_real_T(X_data, X_size);

  // Initialize function input argument 'Y'.
  argInit_d100x6_real_T(Y_data, Y_size);

  // Initialize function input argument 'K'.
  argInit_d100xd100_real_T(K_data, K_size);

  // Call the entry-point 'onlineGPR'.
  onlineGPR(x, y, x_star, gram_matrix_data, gram_matrix_size, X_data, X_size,
            Y_data, Y_size, K_data, K_size, gram_matrix_update_data,
            gram_matrix_update_size, y_star, &cov_star, X_update_data,
            X_update_size, Y_update_data, Y_update_size, K_update_data,
            K_update_size);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  onlineGPR_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_onlineGPR();

  // Terminate the application.
  // You do not need to do this more than one time.
  onlineGPR_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
