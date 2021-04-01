#include <iostream>

#include <g2o/core/g2o_core_api.h>
#include<g2o/core/base_vertex.h>
#include<g2o/core/base_unary_edge.h>
#include<g2o/core/block_solver.h>
#include<g2o/core/solver.h>
#include<g2o/core/linear_solver.h>
#include<g2o/solvers/dense/linear_solver_dense.h>
#include<g2o/core/optimization_algorithm.h>
#include<g2o/core/optimization_algorithm_gauss_newton.h>
#include<g2o/core/optimization_algorithm_levenberg.h>
#include<g2o/core/optimization_algorithm_levenberg.h>


#include <Eigen/Core>
#include<opencv2/core/core.hpp>
#include<cmath>
#include<chrono>

using namespace std;

class CurveFittingVertex : public g2o::BaseVertex<3,Eigen::Vector3d>
{
        public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                virtual void setToOriginImpl();  //待估计的参数重置；

                virtual void oplusImpl(const number_t* v);  // 更新待估计的参数；

                virtual bool read(istream &in){};
                virtual bool write(ostream &out) const{}

};


class CurveFittingEdge : public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
        public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                double _x;

                CurveFittingEdge(double x): BaseUnaryEdge(),_x(x){};  //没有构造函数的类要在参数化列表中进行初始化；

                virtual void computeError(); // 计算单个点的误差；

                virtual void linearizeOplus() ;   // 计算雅格比；

                virtual bool read(istream &in){ }
                virtual bool write(ostream &out) const { }


                
};
