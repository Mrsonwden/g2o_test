#ifndef GENERATE_DATA_H__
#define GENERATE_DATA_H__

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
using namespace std;
using namespace Eigen;

void say_hello();

void generate_data(int nums, double para_a, double para_b, double para_c,  vector<double> &data_x, vector<double> &data_y);
double formular(double para_a, double para_b, double para_c,double x);


#endif
