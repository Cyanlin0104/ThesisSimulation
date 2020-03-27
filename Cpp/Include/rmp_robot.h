#ifndef _RMP_ROBOT_H_
#define _RMP_ROBOT_H_

#include "rmp.h"


enum RobotRMPType{
    Position = 0,
    Orientation
};

class RobotRMPHandler{

public:
    RobotRMPHandler();
    ~RobotRMPHandler();

    virtual bool set_q(const VectorXd& q) = 0;
    virtual bool computeFK_pos(int i, VectorXd& p_des) = 0;
    virtual bool computeFK_ori(int i, VectorXd& o_des) = 0;
    virtual bool computeJacobi_pos(int i, MatrixXd& J_des) = 0;
    virtual bool computeJacobi_ori(int i, MatrixXd& J_des) = 0;
    virtual bool computeJacobi_pos_dot(int i, MatrixXd& J_dot_des) = 0;
    virtual bool computeJacobi_ori_dot(int i, MatrixXd& J_dot_des) = 0;
    virtual int Get_DoF() const = 0;
};


class RobotIntrinsic :public RMPNode{

protected: 
    
    RobotRMPType type;
    RobotRMPHandler* handler;
    // link frame index
    int i; 
    virtual VectorXd psi(const VectorXd&) override;
    virtual MatrixXd J(const VectorXd& y) override;
    virtual MatrixXd J_dot(const VectorXd& y, const VectorXd& y_dot) override;
    virtual void eval() override;

public:

    RobotIntrinsic(std::string name, RMPNode* parent, RobotRMPHandler* handler, int link_frame_idx, RobotRMPType type = RobotRMPType::Position);  
    ~RobotIntrinsic();

};

#endif