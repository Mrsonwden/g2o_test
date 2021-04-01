#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "generate_data.h"

using namespace std;
using namespace Eigen;



//生成 含有高斯噪声的 曲线数据，用于模拟实际采集到的数据；
void generate_data(int nums, double para_a, double para_b, double para_c , vector<double> &data_x, vector<double> &data_y)
{
           double sigma = 1.0;
           cv::RNG rng;
            double y = 0;
            double x = 0;
            for(int  i = 1; i <= nums; i++)
            {
                x = i/(double)nums;
                data_x.push_back(x);
                y = formular(para_a,para_b,para_c ,x);
                y = y + rng.gaussian(sigma);
                data_y.push_back(y);

                // cout << "raw _x : " << x << " raw_y : " <<  exp(para_a * x * x * x + para_b * x * x + para_c * x + para_d)    << "            ||            gs_x : " << x << " gs_y : " <<  y << endl;  
            }            
}

double formular(double para_a, double para_b, double para_c, double x)
{
     return exp(para_a * x * x + para_b * x  + para_c );
}


