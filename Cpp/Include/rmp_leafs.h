#ifndef _RMP_LEAFS_H_
#define _RMP_LEAFS_H_

#include "rmp.h"
#include "scene.h"
#include <functional>



class CollisionAvoidance : public RMPLeaf
{
    /*
    Collision Avoidance RMP leaf
    */
protected:

    double epsilon;
    double alpha;
    double eta; 
    obstacle obs;

    virtual VectorXd psi(const VectorXd& y) override;
    virtual MatrixXd J(const VectorXd& y) override;
    virtual MatrixXd J_dot(const VectorXd& y , const VectorXd& y_dot) override;

    virtual std::tuple<VectorXd, MatrixXd> GDS_func (const VectorXd& x, const VectorXd& x_dot) override;
    
public:
    
    CollisionAvoidance(std::string name, RMPNode* parent, const obstacle& obs, double epsilon=0.2, double alpha=1e-5, double eta=0);

    ~CollisionAvoidance();

};


class GoalAttractorUni :public RMPLeaf{
    /*
    Collision Avoidance RMP leaf
    */

protected:
    double w_u;
    double w_l;
    double sigma;
    double alpha;
    double eta;
    double gain;
    double tol;
    goal g;

    virtual VectorXd psi(const VectorXd& y) override;
    virtual MatrixXd J(const VectorXd& y) override;
    virtual MatrixXd J_dot(const VectorXd& y, const VectorXd& y_dot) override;

    virtual std::tuple<VectorXd, MatrixXd> GDS_func(const VectorXd& x, const VectorXd& x_dot) override;

public:
    GoalAttractorUni(std::string name , RMPNode* parent, const goal& g, double w_u=10, double w_l=1, double sigma=1, double alpha=1, double eta=2, double gain=1, double tol=0.005);
    ~GoalAttractorUni();

};

#endif