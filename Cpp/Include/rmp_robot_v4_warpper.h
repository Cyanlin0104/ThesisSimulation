#ifndef _RMP_ROBOT_V4_WARPPER_H_
#define _RMP_ROBOT_V4_WARPPER_H_

#include "rmp_robot.h"
#include "../Robot/Robot.h"


class RobotRMPHandler_v4 :public RobotRMPHandler{


public:
    Rbt::Robot* robot;
    const double tol = 1e-4;    
public:
    RobotRMPHandler_v4(Rbt::Robot* robot);
    ~RobotRMPHandler_v4();

    virtual bool set_q(const VectorXd& q) override;
    virtual bool computeFK_pos(int i, VectorXd& p_des ) override;
    virtual bool computeFK_ori(int i, VectorXd& o_des ) override;
    virtual bool computeJacobi_pos(int i, MatrixXd& J_pos) override;
    virtual bool computeJacobi_ori(int i, MatrixXd& J_pos) override;
    virtual bool computeJacobi_pos_dot(int i, MatrixXd& J_pos) override;
    virtual bool computeJacobi_ori_dot(int i, MatrixXd& J_pos) override;
    virtual int Get_DoF() const override;
};

#endif