#ifndef  G2O_H__
#define G2O_H__

#include "g2o_test.h"

void CurveFittingVertex::setToOriginImpl()  //待估计的参数重置；
{
        _estimate << 0,0,0;
}
void CurveFittingVertex::oplusImpl(const double* v)  // 更新待估计的参数；因为更新的对象不一定在欧式空间，普通的加法可能并不满足，所以需要在这里重写更新函数；
{
        _estimate += Eigen::Vector3d(v);
}





void CurveFittingEdge::computeError() // 计算单个点的误差；
{
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);  //取得待优化的节点；
        const Eigen::Vector3d abc = v->estimate();                                                                                     //取得待优化节点的值；
        _error(0,0) = _measurement - std::exp(   abc(0,0) * _x * _x +  abc(1,0)  * _x +  abc(2,0)  );  // 计算误差；
}

void CurveFittingEdge::linearizeOplus()    // 计算雅格比；
{
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);    //取得待优化的节点；
        const Eigen::Vector3d abc = v->estimate();                                                                                       // 取得待优化节点的值；
        //计算雅格比；
        double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);                                                                 
        _jacobianOplusXi[0] = -_x * _x *y;
        _jacobianOplusXi[1] = -_x * y;
        _jacobianOplusXi[2] = -y; 
}

#endif