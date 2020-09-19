/**
 * @file SimpleExample.cpp
 * @author Kevin Shou
 * @copyright Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

/*
OSQP solves the convex quadratic optimization problem:

min_x 0.5 * x'Px + q'x
s.t.  l <= Ax <= u

*/

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>

#include <iostream>


int main()
{


    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian; //P or H
    Eigen::VectorXd gradient;  //f or q
    Eigen::SparseMatrix<double> linearMatrix;  //A
    Eigen::VectorXd lowerBound; //l
    Eigen::VectorXd upperBound; //u

//下面的实例来源于 https://ww2.mathworks.cn/help/optim/ug/quadprog.html?s_tid=srchtitle
// 具有线性约束的二次规划
//     hessian.resize(2,2);
//     hessian.insert(0,0) = 1;
//     hessian.insert(1,0) = -1;
//     hessian.insert(0,1) = -1;
//     hessian.insert(1,1) = 2;
//     std::cout << "hessian:" << std::endl << hessian << std::endl;

//     gradient.resize(2);
//     gradient << -2, -6;
    
//     std::cout << "gradient:" << std::endl << gradient << std::endl;

//     linearMatrix.resize(3,2);
//     linearMatrix.insert(0,0) = 1;
//     linearMatrix.insert(0,1) = 1;
//     linearMatrix.insert(1,0) = -1;
//     linearMatrix.insert(1,1) = 2;
//     linearMatrix.insert(2,0) = 2;
//     linearMatrix.insert(2,1) = 1;
//     std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

//     lowerBound.resize(3);
//     lowerBound << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
//     std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

//     upperBound.resize(3);
//     upperBound << 2, 2, 3;
//     std::cout << "upperBound:" << std::endl << upperBound << std::endl;

//     int NumberOfVariables = 2; //A矩阵的列数
//     int NumberOfConstraints = 3; //A矩阵的行数

// 具有线性等式约束的二次规划
//     hessian.resize(2,2);
//     hessian.insert(0,0) = 1;
//     hessian.insert(1,0) = -1;
//     hessian.insert(0,1) = -1;
//     hessian.insert(1,1) = 2;
//     std::cout << "hessian:" << std::endl << hessian << std::endl;

//     gradient.resize(2);
//     gradient << -2, -6;
    
//     std::cout << "gradient:" << std::endl << gradient << std::endl;

//     linearMatrix.resize(1,2);
//     linearMatrix.insert(0,0) = 1;
//     linearMatrix.insert(0,1) = 1;
//     std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

//     lowerBound.resize(1);
//     lowerBound << 0;
//     std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

//     upperBound.resize(1);
//     upperBound << 0;
//     std::cout << "upperBound:" << std::endl << upperBound << std::endl;

//     int NumberOfVariables = 2; //A矩阵的列数
//     int NumberOfConstraints = 1; //A矩阵的行数


//具有线性约束和边界的二次最小化
    hessian.resize(3,3);
    hessian.insert(0,0) = 1;
    hessian.insert(1,0) = -1;
    hessian.insert(2,0) = 1;
    hessian.insert(0,1) = -1;
    hessian.insert(1,1) = 2;
    hessian.insert(2,1) = -2;
    hessian.insert(0,2) = 1;
    hessian.insert(1,2) = -2;
    hessian.insert(2,2) = 4;
    std::cout << "hessian:" << std::endl << hessian << std::endl;

    gradient.resize(3);
    gradient << 2, -3, 1;
    
    std::cout << "gradient:" << std::endl << gradient << std::endl;

    linearMatrix.resize(4,3);
    linearMatrix.insert(0,0) = 1;
    linearMatrix.insert(1,0) = 0;
    linearMatrix.insert(2,0) = 0;
    linearMatrix.insert(3,0) = 1;

    linearMatrix.insert(0,1) = 0;
    linearMatrix.insert(1,1) = 1;
    linearMatrix.insert(2,1) = 0;
    linearMatrix.insert(3,1) = 1;

    linearMatrix.insert(0,2) = 0;
    linearMatrix.insert(1,2) = 0;
    linearMatrix.insert(2,2) = 1;
    linearMatrix.insert(3,2) = 1;
    std::cout << "linearMatrix:" << std::endl << linearMatrix << std::endl;

    lowerBound.resize(4);
    lowerBound << 0, 0, 0, 0.5;
    std::cout << "lowerBound:" << std::endl << lowerBound << std::endl;

    upperBound.resize(4);
    upperBound << 1, 1, 1, 0.5;
    std::cout << "upperBound:" << std::endl << upperBound << std::endl;

    int NumberOfVariables = 3; //A矩阵的列数
    int NumberOfConstraints = 4; //A矩阵的行数



    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
    solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
    if(!solver.data()->setHessianMatrix(hessian)) return 1;//设置P矩阵
    if(!solver.data()->setGradient(gradient)) return 1; //设置q or f矩阵。当没有时设置为全0向量
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
    if(!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
    if(!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界

    // instantiate the solver
    if(!solver.initSolver()) return 1;

    Eigen::VectorXd QPSolution;

    // solve the QP problem
    if(!solver.solve()) return 1;

    // get the controller input
    QPSolution = solver.getSolution();

    std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;

    return 0;
}
