#include <iostream>
#include <fstream>
using namespace std;
class StateRecorder{
    public:
        /*fstream to record the state for debug*/
        ofstream f_x_v, f_y, f_x_star, f_y_star, f_X, f_Y, f_K, f_gram;
    StateRecorder(){
        f_x_v.open ("/home/jjhu/matlab_ws/f_x_v.txt");
        f_y.open ("/home/jjhu/matlab_ws/f_y.txt");
        f_x_star.open ("/home/jjhu/matlab_ws/f_x_star.txt");
        f_y_star.open ("/home/jjhu/matlab_ws/f_y_star.txt");
        f_X.open ("/home/jjhu/matlab_ws/f_X.txt");
        f_Y.open ("/home/jjhu/matlab_ws/f_Y.txt");
        f_K.open ("/home/jjhu/matlab_ws/f_K.txt");
        f_gram.open ("/home/jjhu/matlab_ws/f_gram.txt");
    }
    void record(double x_v[6],double y[6],double x_star[6],double y_star[6],double X[],double Y[],double K[],double gram[]){
        for(int i=0;i<6;i++)
            f_x_v<<x_v[i]<<' ';
        f_x_v<<'\n';
        
        for(int i=0;i<6;i++)
            f_y<<y[i]<<' ';
        f_y<<'\n';
        
        for(int i=0;i<6;i++)
            f_x_star<<x_star[i]<<' ';
        f_x_star<<'\n';
        
        for(int i=0;i<6;i++)
            f_y_star<<y_star[i]<<' ';
        f_y_star<<'\n';
        
        for(int i=0;i<1;i++)
            f_X<<X[i]<<' ';
        f_X<<'\n';
        
        for(int i=0;i<1;i++)
            f_Y<<Y[i]<<' ';
        f_Y<<'\n';
        
        for(int i=0;i<1;i++)
            f_K<<K[i]<<' ';
        f_K<<'\n';
        
        for(int i=0;i<1;i++)
            f_gram<<gram[i]<<' ';
        f_gram<<'\n';
      
    }
    void close(){
        f_x_v.close();
        f_y.close();
        f_x_star.close();
        f_y_star.close();
        f_X.close();
        f_Y.close();
        f_K.close();
        f_gram.close();
    }
    ~StateRecorder(){
        f_x_v.close();
        f_y.close();
        f_x_star.close();
        f_y_star.close();
        f_X.close();
        f_Y.close();
        f_K.close();
        f_gram.close();
    }
    
};
