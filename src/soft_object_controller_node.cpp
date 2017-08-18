//system and self
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "onlineGPR/rt_nonfinite.h"
#include "onlineGPR/onlineGPR.h"
#include "onlineGPR/rt_nonfinite.h"
#include "onlineGPR/rtwtypes.h"
#include "onlineGPR/onlineGPR_types.h"
#include "onlineGPR/onlineGPR_terminate.h"
#include "onlineGPR/onlineGPR_initialize.h"
#include "StateRecorder.h"


//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/chain.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <signal.h>


#define N 6
#define M 6
#define MEMORY_SIZE 100
#define FPS 10
#define MAX_VEL 0.06  //max allowed velocity
#define TOLERANCE 0.006
#define CONTROL_GAIN 2

//real cartesian velocity of ee reciving from yumi topic
double cart_v_real[M] = {0};
//global variable feedback points
double p[N] = {0};

bool received = false,getFeedbackPoint = false,moved = false;


double norm(const double x[3])
{
  double y;
  double scale;
  int k;
  double absxk;
  double t;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 0; k < 3; k++) {
    absxk = std::abs(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0 + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}
// static void getFeatures(const double p[6],double c_p[4])
// {
//     int i,i0;
//     double b_p[3];
//     for (i = 0; i < 3; i++) {
//         b_p[i] = p[i] - p[i + 3];
//     }
// 
//     for (i0 = 0; i0 < 3; i0++) {
//         c_p[i0] = (p[i0] + p[3 + i0]) / 2.0;
//     }
// 
//     c_p[3] = norm(b_p);
// 
// }
static void getFeatures(const double p[N],double c_p[N])
{
    double temp;
    for(int i=0;i<N;i++)
        c_p[i] = p[i];
    if(c_p[0] > c_p[3])
    {
        for(int i=0;i<3;i++)
        {
            temp = c_p[i];
            c_p[i] = c_p[i+3];
            c_p[i+3] = temp;
        }  
    }

    

}
//return value higher than threshold
void highPass(double *pointer,int size, double threshold)
{
    for(int i=0;i<size;i++){
        if(std::abs(pointer[i])<threshold)
            pointer[i] = 0;
    }
}




//callback to get joint state
void cartVelCallback(const std_msgs::Float64MultiArray& msg)
{
    for (int i = 0; i < msg.data.size(); i++) {
        cart_v_real[i] = msg.data[i];
    }
    //high pass to filter noise
    highPass(cart_v_real,msg.data.size(),1e-4);
    
    for(int i=0;i<6;i++)
        if(cart_v_real[i] != 0)
            moved = true;
//     std::cout<<cart_v_real[0]<<"\n";
    received = true;
    
    // std::cout << "received real cartesian velocity from yumi topic\n";

}


//callback to get joint state
void feedbackCallback(const std_msgs::Float64MultiArray& msg)
{
    for (int i = 0; i < msg.data.size(); i++) {
        p[i] = msg.data[i];
    }
//     std::cout<<p[0]<<"\n";
    getFeedbackPoint = true;

}

bool start = false;
void startCallback(const std_msgs::Bool& msg)
{
    start = msg.data;
}

//for state recorder 
StateRecorder stateRecorder;
void mySigintHandler(int sig)
{
    stateRecorder.close();
    ros::shutdown();
}
void main_soft_object_manipulator(int argc,char ** argv)
{

    //onlineGPR input variable
    double x[N] = {0};
    double x_v[N] = {0};
    double x_old[N] = {0};
    double y[M] = {0};
    double x_star[N] = {0};
    static double gram_matrix_data[MEMORY_SIZE*MEMORY_SIZE] = {0};
    int gram_matrix_size[2] = {0};
    double X_data[MEMORY_SIZE*N] = {0};
    int X_size[2] = {0};
    double Y_data[MEMORY_SIZE*M] = {0};
    int Y_size[2] = {0};
    static double K_data[MEMORY_SIZE*MEMORY_SIZE] = {0};
    int K_size[2] = {0};
    static double gram_matrix_update_data[(MEMORY_SIZE+1)*(MEMORY_SIZE+1)] = {0};
    int gram_matrix_update_size[2] = {0};
    double y_star[M] = {0};
    double cov_star;
    double X_update_data[(MEMORY_SIZE+1)*N] = {0};
    int X_update_size[2] = {0};
    double Y_update_data[(MEMORY_SIZE+1)*M] = {0};
    int Y_update_size[2] = {0};
    static double K_update_data[(MEMORY_SIZE+1)*(MEMORY_SIZE+1)] = {0};
    int K_update_size[2] = {0};
        
    //target
    double xd[N] = {0.010599430650472641, -0.021574804559350014, 0.4256342053413391, 0.060182858258485794, 0.0037090808618813753, 0.4125695824623108};
    
    ros::init(argc,argv, "soft_object_controller", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigintHandler);
    ros::NodeHandle node;
    //subscrib topic /yumi/joint_state to get joint state
    ros::Subscriber subCartVel = node.subscribe("/yumi/ikSloverVel_controller/state", 2, cartVelCallback);
    //subscrib the feedback point
    ros::Subscriber subFeedbackPoint = node.subscribe("/soft_object_tracking/centroid", 2, feedbackCallback);
    //subscrib the start signal
    ros::Subscriber subStart = node.subscribe("/yumi/ikSloverVel_controller/go", 2, startCallback);
    //publish velocity control command
    ros::Publisher cartVelPublisher = node.advertise<std_msgs::Float64MultiArray> ("/yumi/ikSloverVel_controller/command", 1);
    
    //command
    std_msgs::Float64MultiArray armCommand;
    armCommand.data.resize(M);
    

    ros::Rate rate(FPS);
    bool first_time = true;
    bool stopFlag = false;
    int count = 0;
    while(ros::ok())
    {
        //reached the target
        if(stopFlag){
            if(start){
                count++;
                for(int i=0;i<M;i++)
                    armCommand.data[i] = 0;
                cartVelPublisher.publish(armCommand);
            }
            std::cout << "Reached the target SUCCESSFULLY\n";
            //after 0.5 second
            if(count/FPS > 0.5 && count/FPS < 1.5 ){
                for(int i=0;i<M;i++)
                    armCommand.data[i] = 0;
                armCommand.data[2] = -0.06;
                armCommand.data[5] = -0.06;
                cartVelPublisher.publish(armCommand);
            }
            else if(count/FPS >= 1.5 && count/FPS < 2 ){
                for(int i=0;i<M;i++)
                    armCommand.data[i] = 0;
                cartVelPublisher.publish(armCommand);
            }
            else if(count/FPS >= 2)
                 break;

                
            
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        
        if(received && getFeedbackPoint){
            for(int i=0;i<N;i++)
                x_old[i] = x[i];
            getFeatures(p,x);
//             for(int i=0;i<N;i++)
//                 x[i] = p[i];
            for(int i=0;i<N;i++)
                x_v[i] = (x[i] - x_old[i])*FPS;
            for(int i=0;i<N;i++)
                x_star[i] = xd[i] - x[i];
            for(int i=0;i<M;i++)
                y[i] = cart_v_real[i];
            highPass(y,M,1e-4);
//             if(first_time){
//                 double C[N] = {0,-0.05,0,0};
//                 for(int i=0;i<N;i++)
//                     xd[i] = x[i] + C[i];

//             }
            
            if(moved){
                if(first_time){
                    /*Initialize*/
                    //X
                    for(int i=0;i<N;i++)
                        X_data[i] = x_v[i];
                    X_size[0] = N;
                    X_size[1] = 1;
                    //Y
                    for(int i=0;i<M;i++)
                        Y_data[i] = y[i];
                    Y_size[0] = 1;
                    Y_size[1] = M;
                    
                    double keps = 0.001;
                    double F = 1;
                    //K
                    K_data[0] = F*F;
                    K_size[0] = 1;
                    K_size[1] = 1;
                    //gram_matrix
                    gram_matrix_data[0] = 1/(F*F+keps);
                    gram_matrix_size[0] = 1;
                    gram_matrix_size[1] = 1;
                    
                    stateRecorder.record(x_v,y,x_star,y_star,X_data,Y_data,K_data,gram_matrix_data);
//                     for(int i=0;i<N;i++)
//                         target_file<<xd[i]<<' ';
                    
                    first_time = false;
                }

                else {
                    // Call the entry-point 'onlineGPR'.
                    onlineGPR(x_v, y, x_star, gram_matrix_data, gram_matrix_size, X_data, X_size,
                        Y_data, Y_size, K_data, K_size, gram_matrix_update_data,
                        gram_matrix_update_size, y_star, &cov_star, X_update_data,
                        X_update_size, Y_update_data, Y_update_size, K_update_data,
                        K_update_size);
                    stateRecorder.record(x_v,y,x_star,y_star,X_data,Y_data,K_data,gram_matrix_data);
                    //update K and gram_matrix
                    K_size[0] = K_update_size[0];
                    K_size[1] = K_update_size[1];
                    gram_matrix_size[0] = gram_matrix_update_size[0];
                    gram_matrix_size[1] = gram_matrix_update_size[1];
                    for(int i=0;i<K_size[1];i++){
                        for(int j=0;j<K_size[0];j++){
                            K_data[i*K_size[0]+j] = K_update_data[i*K_size[0]+j];
                            gram_matrix_data[i*K_size[0]+j] = gram_matrix_update_data[i*K_size[0]+j];
                        }
                    }
                    
                    //update X
                    X_size[0] = X_update_size[0];
                    X_size[1] = X_update_size[1];
                    for(int i=0;i<X_size[1];i++)
                        for(int j=0;j<X_size[0];j++)
                            X_data[i*X_size[0]+j] = X_update_data[i*X_size[0]+j];
                    
                    //update Y
                    Y_size[0] = Y_update_size[0];
                    Y_size[1] = Y_update_size[1];
                    for(int i=0;i<Y_size[1];i++)
                        for(int j=0;j<Y_size[0];j++)
                            Y_data[i*Y_size[0]+j] = Y_update_data[i*Y_size[0]+j];
                    
                }
            }
//             std::cout<<"command v: ";
//             for(int i=0;i<6;i++)
//                 std::cout<< y_star[i]<<" ";

            received = false;
            getFeedbackPoint = false;
            moved = false;
            
            //check if reach the desired vaule
            stopFlag = true;
            std::cout<<"deltX: ";
            for(int i=0;i<N;i++){
                if(std::abs(xd[i] - x[i]) > TOLERANCE)
                    stopFlag = false;
                std::cout<<xd[i] - x[i]<<' ';
            }
            std::cout << "\n";
            
            std::cout << "Velocity: ";
            double y_star_gain[M] = {0};
            for(int i=0;i<M;i++){
                y_star_gain[i] = y_star[i] * CONTROL_GAIN;
                y_star_gain[i] = y_star_gain[i]<MAX_VEL?y_star_gain[i]:MAX_VEL;
                y_star_gain[i] = y_star_gain[i]>-MAX_VEL?y_star_gain[i]:-MAX_VEL;
                if(stopFlag)
                    armCommand.data[i] = 0;
                else
                    armCommand.data[i] = y_star_gain[i];
                std::cout<<armCommand.data[i]<<' ';
                
            }
            std::cout << "\n";
            
            if(start){
                //send arm velocity
                cartVelPublisher.publish(armCommand);
                std::cout << "sended velocity command\n";
            }
            std::cout << "\n-----------------------\n";
            
            }
            else{
                for(int i=0;i<M;i++)
//                     cart_v[i] = 0;
                std::cout << "Didn't receive the cart_v_real or get feedback points\n";
            }

            ros::spinOnce();
            rate.sleep();
        }
}


int main(int argc, char ** argv)
{
    // Initialize the application.
    // You do not need to do this more than one time.
    onlineGPR_initialize();

    // Invoke the entry-point functions.
    // You can call entry-point functions multiple times.
    main_soft_object_manipulator(argc,argv);

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

