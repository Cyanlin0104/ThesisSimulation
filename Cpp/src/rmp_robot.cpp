#include "../Include/rmp_robot.h"

RobotRMPHandler::RobotRMPHandler(){

}

RobotRMPHandler::~RobotRMPHandler(){

}


RobotIntrinsic::RobotIntrinsic(std::string name, 
RMPNode* parent,
RobotRMPHandler* handler,
int link_frame_idx,
RobotRMPType type)
:RMPNode(name, parent, 3),
type(type),
handler(handler),
i(link_frame_idx)
{
    if(i > handler->Get_DoF() - 1)
        std::cout << "Error: link index out of range" << std::endl;
}

RobotIntrinsic::~RobotIntrinsic(){

}

VectorXd RobotIntrinsic::psi(const VectorXd& y){
    handler->set_q(y);
    VectorXd x(3);
    if (type == RobotRMPType::Position)
        handler->computeFK_pos(i, x);
    if (type == RobotRMPType::Orientation)
        handler->computeFK_ori(i, x);
    return x;
}

MatrixXd RobotIntrinsic::J(const VectorXd& y){
    handler->set_q(y);
    MatrixXd J_ret(3, handler->Get_DoF());
    if (type == RobotRMPType::Position)
        handler->computeJacobi_pos(i, J_ret);
    if (type == RobotRMPType::Orientation)
        handler->computeJacobi_ori(i, J_ret);
    return J_ret;
}

MatrixXd RobotIntrinsic::J_dot(const VectorXd& y, const VectorXd& y_dot){
    handler->set_q(y);
    MatrixXd J_dot_ret(3, handler->Get_DoF());
    if (type == RobotRMPType::Position)
        handler->computeJacobi_pos(i, J_dot_ret);
    if (type == RobotRMPType::Orientation)
        handler->computeJacobi_pos(i, J_dot_ret);
    
    return J_dot_ret;
}

void RobotIntrinsic::eval(){
    // do nothing
}