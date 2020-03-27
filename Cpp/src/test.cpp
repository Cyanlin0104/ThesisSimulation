#include "../Include/rmp.h"
#include "../Include/rmp_leafs.h"
#include "../Include/rmp_robot.h"
#include "../Include/rmp_robot_v4_warpper.h"
#include "../Robot/NTU_ArmHand_Parameter.h"

int main(){


// Robot init
Rbt::Robot KArm;
NTU_Arm::Build_KArm(KArm);

//create handler 
RobotRMPHandler_v4* KArm_handler = new RobotRMPHandler_v4(&KArm);
//
obstacle obs = {VectorXd::Zero(3), 1.0};
goal go(3);


RMPRoot* r = new RMPRoot("r", 6);

RobotIntrinsic* RoboInc = new RobotIntrinsic("RoboInc", r, KArm_handler, 5, RobotRMPType::Position);

CollisionAvoidance* Cave = new CollisionAvoidance("Cave", RoboInc, obs);
GoalAttractorUni* Gaoi = new GoalAttractorUni("Gaoi", RoboInc, go);

RMPTree tree(r, {Cave, Gaoi, RoboInc});

VectorXd q_test(6), q_dot_test(6);
q_test << 0, 0, 0, 0, 0, 0;
q_dot_test << 0, 0, 0, 0, 0, 0;

VectorXd x_ddot = tree.solve(q_test, q_dot_test);
std::cout << "solved a: \n" << x_ddot << std::endl;


delete r;
delete Cave;
delete Gaoi;
delete KArm_handler;
delete RoboInc;
return 0;
}
