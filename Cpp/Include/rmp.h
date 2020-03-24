#ifndef _RMP_H_
#define _RMP_H_

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <tuple>

using namespace Eigen;


class RMPNode
{
 /*
 A Generic RMP node
 */ 

protected:
    const int dim;
    std::string name;
    RMPNode* parent;
    std::vector<RMPNode*> children;

    // VectorXd (*RMPNode::psi)(const VectorXd& y);
    // /*
    // gradient of psi, Jacobi Matrix
    // */
    // MatrixXd (*RMPNode::J)(const VectorXd& y);
    
    // MatrixXd (*RMPNode::J_dot)(const VectorXd& y , const VectorXd& y_dot);
  

    /* State x */
    VectorXd x;
    VectorXd x_dot;
    
    /* desired force */
    VectorXd f;
    /* inertia matrix*/
    MatrixXd M;
    

public:
    RMPNode(std::string name, 
            RMPNode* parent, 
            int dim);
    
    ~RMPNode();

    void add_child(RMPNode* node);
    void clear_children();

    VectorXd get_x() const{return x;}
    VectorXd get_x_dot() const{return x_dot;}
    int get_dim() const{return dim;}
    RMPNode* get_parent() const{return parent;}
    std::string get_name() const{return name;}

    /*
    psi : mapping function psi: y -> x
    where y is the coordinate of Manifold corresponding to parent RMP, and x is the coordinate of Manifold corresponding to this RMP.
    */
    virtual VectorXd psi(const VectorXd& y) = 0;
    virtual MatrixXd J(const VectorXd& y) = 0;
    virtual MatrixXd J_dot(const VectorXd& y , const VectorXd& y_dot) = 0;

    virtual void eval() = 0;


friend class RMPTree;

};

class RMPRoot :public RMPNode
{
    /* 
    A root node
    */

protected:

    virtual void eval() override;
    virtual VectorXd psi(const VectorXd& y) override;
    virtual MatrixXd J(const VectorXd& y) override;
    virtual MatrixXd J_dot(const VectorXd& y , const VectorXd& y_dot) override;


public:
    RMPRoot(std::string _name, int _dim);
    ~RMPRoot();
};


class RMPLeaf :public RMPNode
{
    /*
    A leaf node
    */

protected:

    virtual std::tuple<VectorXd, MatrixXd> GDS_func (const VectorXd&, const VectorXd&) = 0;

    virtual void eval() override;

public:
    
    RMPLeaf(std::string name, 
            RMPNode* parent, 
            int dim
    );

    ~RMPLeaf();
};


class RMPTree
{
protected:
    RMPNode* root;
    void add_node(RMPNode* node);

public:
    RMPTree();
    
    RMPTree(RMPNode* root, const std::vector<RMPNode*>& nodes);
    ~RMPTree();
    
    static void set_state(RMPNode* node, const VectorXd& x, const VectorXd& x_dot);

    void pushforward(RMPNode* node);
    void pullback(RMPNode* node);
    
    // resolve the natural-form RMP
    VectorXd resolve(RMPNode* node);

    // solve the 
    VectorXd solve(const VectorXd& x, const VectorXd& x_dot);
};


#endif