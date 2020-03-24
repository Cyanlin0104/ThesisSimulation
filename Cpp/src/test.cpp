#include "../Include/rmp.h"
#include "../Include/rmp_leafs.h"



int main(){

obstacle obs = {VectorXd::Zero(3), 1.0};
goal go(3);

RMPRoot* r = new RMPRoot("r", 3);


CollisionAvoidance* Cave = new CollisionAvoidance("Cave", r, obs);
GoalAttractorUni* Gaoi = new GoalAttractorUni("Gaoi", r, go);

RMPTree tree(r, {Cave, Gaoi});
VectorXd x_test(3), x_dot_test(3);
x_test << 10, 0, 5;
x_dot_test << -5, 0, 5;

VectorXd x_ddot = tree.solve(x_test, x_dot_test);
std::cout << "solved a: \n" << x_ddot << std::endl;


delete r;
delete Cave;

return 0;
}
