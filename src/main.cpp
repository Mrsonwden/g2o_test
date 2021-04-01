#include <iostream>

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

#include <g2o_test.h>
#include <generate_data.h>

using namespace std;


int main()
{
        cout << "hello world " << endl;
        //生成伪数据；
	int total_data = 10000;
	double a_t = 5;
	double b_t = 1;
	double c_t = 1;
	vector<double> data_x;
	vector<double> data_y;
	 generate_data(total_data, a_t, b_t, c_t , data_x, data_y);
        //设定ae,be,ce,de 的初值；
	double a_e = 15;
	double b_e = 20;
	double c_e  =10;
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1> > BlockSloverType;
        typedef g2o::LinearSolverDense<BlockSloverType::PoseMatrixType> LinearSolverType;
        //使用求解器初始化优化器；
        auto solver = new   g2o::OptimizationAlgorithmLevenberg  (
                                                        g2o::make_unique<BlockSloverType>(
                                                                g2o::make_unique<LinearSolverType>()
                                                        )
                                                );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(true);

        //增加顶点；
        CurveFittingVertex *v = new CurveFittingVertex();
        v->setEstimate(Eigen::Vector3d(a_e,b_e,c_e));
        v->setId(0);
        optimizer.addVertex(v);

        //增加边；
        for(int i = 0; i < data_x.size(); i++)
        {
                CurveFittingEdge * e = new CurveFittingEdge(data_x[i]);
                
                e->setVertex(0,v);
                e->setId(1 +i);
                e->setMeasurement(data_y[i]);   //设置观测值；
                e->setInformation(Eigen::Matrix<double,1,1>::Identity() );  //设置信息矩阵；
                optimizer.addEdge(e);
        }

       //开始优化；
        optimizer.initializeOptimization();
        optimizer.optimize(1000);

        //输出结果；
        Eigen::Vector3d abc_estimate = v->estimate();
        cout << "estimate abc " << abc_estimate.transpose() << endl;
        
        
        return 0;
}