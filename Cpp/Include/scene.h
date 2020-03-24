#ifndef _SCENE_H_
#define _SCENE_H_

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

struct obstacle{
    // center postion
    VectorXd x;
    // radius
    double r;
};

class goal{
    // target position & velocity

protected:
    VectorXd x;
    VectorXd x_dot;
    const int dim;

public:
    
    goal(int dim):
    dim(dim),
    x(VectorXd::Zero(dim)),
    x_dot(VectorXd::Zero(dim))
    {
    }
    
    goal(const VectorXd& x)
    :x(x),
    dim(x.size()),
    x_dot(VectorXd::Zero(x.size()))
    {
        
    }

    goal(const VectorXd& x, const VectorXd& x_dot)
    :x(x),
    x_dot(x_dot),
    dim(x.size())
    {
    }

    VectorXd get_x() const{
        return x;
    }
    VectorXd get_x_dot() const{
        return x_dot;
    }
    int get_dim() const{
        return dim;
    }
};


class scene
{
/*
A scene includes obstacles, goals and the robot status

this class provides methods that update the status information of whole operational space

*/

protected:
std::vector<obstacle> obstacle_list;
std::vector<goal> goal_list;
int num_goals;

public:
scene(){
    obstacle_list.clear();
    goal_list.clear();
}

// scene(const std::vector<goal>& _goal_list, const std::vector<obstacle>& _obstacle_list){
//     obstacle_list = _obstacle_list;
//     goal_list = _goal_list;
// }
~scene();

};

#endif