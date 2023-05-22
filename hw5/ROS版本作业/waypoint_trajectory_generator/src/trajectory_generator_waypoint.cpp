#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */


    /*   Produce the dereivatives in X, Y and Z axis directly.  */
    

    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
    //cout<<"------------------------0-----------------"<<endl;
    MatrixXd Q=MatrixXd::Zero(p_num1d*m,p_num1d*m);  
    for(int k = 0;k<m;k++)
    {  
        for(int i=4;i<p_num1d;i++)
        {
            for(int j=4;j<p_num1d;j++)
            {
                double zz=Factorial(i)/Factorial(i-4)*Factorial(j)/Factorial(j-4);
                Q(p_num1d*k+i,p_num1d*k+j)=zz/((i+j)-7)*pow(Time(k),i+j-7);
            }
        }
    }
    
    //cout<<"------------------------1-----------------"<<endl;
    MatrixXd M=MatrixXd::Zero(p_num1d*m,p_num1d*m); 
    for(int k = 0;k<m;k++)
    { 
        int tt=p_num1d*k;
        M(tt+0,tt+0)=1;
        M(tt+1,tt+1)=1;
        M(tt+2,tt+2)=2;
        M(tt+3,tt+3)=6;
        for(int i=4;i<p_num1d;i++)
        {
            for(int j=i-4;j<p_num1d;j++)
            {
                M(tt+i,tt+j)=pow(Time(k),j-(i-4))*Factorial(j)/Factorial(j-i+4);
            }
        }
    }
    
    //cout<<"------------------------2-----------------"<<endl;
    MatrixXd Ct=MatrixXd::Zero(p_num1d*m,4*m+4); 
    for(int i=0;i<4;i++)
        Ct(i,i)=1;
    for(int k=1;k<m;k++)
    {
        Ct(8*k+1-5,4+k-1)=1;
        Ct(8*k+2-5,m+7+3*(k-1)+0)=1;
        Ct(8*k+3-5,m+7+3*(k-1)+1)=1;
        Ct(8*k+4-5,m+7+3*(k-1)+2)=1;
   
        Ct(8*k+5-5,4+k-1)=1;
        Ct(8*k+6-5,m+7+3*(k-1)+0)=1;
        Ct(8*k+7-5,m+7+3*(k-1)+1)=1;
        Ct(8*k+8-5,m+7+3*(k-1)+2)=1;
    }
    for(int i=0;i<4;i++)
        Ct(p_num1d*m-4+i,m+3+i)=1;
    
    
    MatrixXd C = Ct.transpose();
    MatrixXd R,R_cell,R_pp,R_fp;
    R = C * (M.inverse()).transpose()* Q * M.inverse() * Ct;
    R_pp = R.block(m+7, m+7, 3*(m-1), 3*(m-1));
    R_fp = R.block(0, m+7, m+7, 3*(m-1));
    //cout<<"------------------------3-----------------"<<endl;


    MatrixXd dFx=MatrixXd::Zero(m+7,1);
    dFx(0)=Path(0,0);
    dFx(1)=Vel(0,0);
    dFx(2)=Acc(0,0);
    dFx(3)=0;
    dFx(m+3)=Path(m,0);
    for(int i=1;i<m;i++)
       dFx(3+i)= Path(i,0);
    
    MatrixXd dFy=MatrixXd::Zero(m+7,1);
    dFy(0)=Path(0,1);
    dFy(1)=Vel(0,1);
    dFy(2)=Acc(0,1);
    dFy(3)=0;
    dFy(m+3)=Path(m,1);
    for(int i=1;i<m;i++)
       dFy(3+i)= Path(i,1);

    MatrixXd dFz=MatrixXd::Zero(m+7,1);
    dFz(0)=Path(0,2);
    dFz(1)=Vel(0,2);
    dFz(2)=Acc(0,2);
    dFz(3)=0;
    dFz(m+3)=Path(m,2);
    for(int i=1;i<m;i++)
       dFz(3+i)= Path(i,2);


    //cout<<"------------------------4-----------------"<<endl;
    MatrixXd dPx = -R_pp.inverse() * R_fp.transpose() * dFx;   
    MatrixXd dx=MatrixXd::Zero(4*m+4,1);
    dx.topRows(m+7)=dFx;
    dx.middleRows(m+7, 3*(m-1))=dPx;
    Px = M.inverse() * Ct * dx;
    
    MatrixXd dPy = -R_pp.inverse() * R_fp.transpose() * dFy;
    MatrixXd dy=MatrixXd::Zero(4*m+4,1);
    dy.topRows(m+7)=dFy;
    dy.middleRows(m+7, 3*(m-1))=dPy;
    Py = M.inverse() * Ct * dy;
    
    MatrixXd dPz = -R_pp.inverse() * R_fp.transpose() * dFz;
    MatrixXd dz=MatrixXd::Zero(4*m+4,1);
    dz.topRows(m+7)=dFz;
    dz.middleRows(m+7, 3*(m-1))=dPz;
    Pz = M.inverse() * Ct * dz;
    
    //cout<<"------------------------5-----------------"<<endl;
    for(int i=0;i<m;i++)
    {
        for(int j=0;j<p_num1d;j++)
        {
            PolyCoeff(i,j)=Px(i*p_num1d+j);
            PolyCoeff(i,j+p_num1d)=Py(i*p_num1d+j);
            PolyCoeff(i,j+p_num1d*2)=Pz(i*p_num1d+j);
        }
    }
    //cout<<"------------------------6-----------------"<<endl;
    return PolyCoeff;
}
