#include "../Include/rmp_robot_v4_warpper.h"


RobotRMPHandler_v4::RobotRMPHandler_v4(Rbt::Robot* robot)
:robot(robot)
{

}

RobotRMPHandler_v4::~RobotRMPHandler_v4()
{

}
int RobotRMPHandler_v4::Get_DoF() const{
    return robot->DOF_Size();
}

bool RobotRMPHandler_v4::set_q(const VectorXd& q)
{  
    // check if it is the same command send to each link RMP
    if((q - robot->Get_q()).norm() > tol)
    robot->Set_q(q);
    return true;
}


bool RobotRMPHandler_v4::computeFK_pos(int i,VectorXd& x_des){ 
    Rbt::DHFrame* frame_i = robot->Get_ActiveFrame()[i];
    VectorXd x(3);
    x_des = frame_i->Get_TFMat().block<3,1>(0,3);
    return true;
}

bool RobotRMPHandler_v4::computeFK_ori(int i,VectorXd& x_des){
    Rbt::DHFrame* frame_i = robot->Get_ActiveFrame()[i];
    VectorXd x(3);
    x_des = Rbt::Get_Orientation(frame_i->Get_TFMat().block<3,3>(0,0));
    return true;
}

bool RobotRMPHandler_v4::computeJacobi_pos(int i,MatrixXd& J_des){
    MatrixXd J(6, Get_DoF());
    robot->Get_Jacobian(J);
    J_des = J.block(0, 0, 3, Get_DoF());  
    return true;
}

bool RobotRMPHandler_v4::computeJacobi_ori(int i, MatrixXd& J_des){
    MatrixXd J(6, Get_DoF());
    robot->Get_Jacobian(J);
    J_des = J.block(3, 0, 3, Get_DoF());  
    return true;
}

bool RobotRMPHandler_v4::computeJacobi_pos_dot(int i, MatrixXd& J_dot_des){
    J_dot_des = MatrixXd::Zero(3, Get_DoF());
    return true;
}

bool RobotRMPHandler_v4::computeJacobi_ori_dot(int i, MatrixXd& J_dot_des){
    J_dot_des = MatrixXd::Zero(3, Get_DoF());
    return true;
}