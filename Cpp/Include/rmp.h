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

public:
    int dim;
    std::string name;

protected:

    RMPNode* parent;
    std::vector<RMPNode*> children;

    /*
    psi : mapping function psi: y -> x
    where y is the coordinate of Manifold corresponding to parent RMP, and x is the coordinate of Manifold corresponding to this RMP.
    */
    VectorXd (*psi)(const VectorXd& y);
    /*
    gradient of psi, Jacobi Matrix
    */
    MatrixXd (*J)(const VectorXd& y);
    
    MatrixXd (*J_dot)(const VectorXd& y , const VectorXd& y_dot);
  

    /* State x */
    VectorXd x;
    VectorXd x_dot;
    
    /* desired force */
    VectorXd f;
    /* inertia matrix*/
    VectorXd M;
    

public:
    RMPNode();
    
    RMPNode(std::string name, 
            RMPNode* parent, 
            int dim, 
            VectorXd (*psi)(const VectorXd&), 
            MatrixXd (*J)(const VectorXd&), 
            MatrixXd (*J_dot)(const VectorXd&, const VectorXd&));
    
    ~RMPNode();
public:
    void add_child(RMPNode* node);

    void clear_children();
    virtual void eval();


friend class RMPTree;
};

class RMPRoot :public RMPNode
{
    /* 
    A root node
    */

public:
    RMPRoot();
    RMPRoot(std::string _name, int _dim);
    virtual void eval();
};


class RMPLeaf :public RMPNode
{
    /*
    A leaf node
    */

protected:

    std::tuple<VectorXd, MatrixXd> (*RMP_func)(VectorXd, VectorXd);

public:

    RMPLeaf();
    
    RMPLeaf(std::string name, 
            RMPNode* parent, 
            int dim,
            VectorXd (*psi)(const VectorXd&), 
            MatrixXd (*J)(const VectorXd&), 
            MatrixXd (*J_dot)(const VectorXd&, const VectorXd&),
            std::tuple<VectorXd, MatrixXd> RMP_func(VectorXd, VectorXd)
    );
   
   virtual void eval();


};


class RMPTree
{
protected:
    RMPNode* root;
    void add_node(RMPNode* node);

public:
    RMPTree();
    
    RMPTree(RMPNode* root, std::vector<RMPNode*> nodes);
    ~RMPTree();
    
    static void set_state(RMPNode* node, VectorXd x, VectorXd x_dot);

    void pushforward(RMPNode* node);
    void pullback(RMPNode* node);
    void resolve(RMPNode* node);
    void solve(RMPNode* node);
};