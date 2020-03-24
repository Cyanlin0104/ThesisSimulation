#include "../Include/rmp.h"

//#define debug

RMPNode::RMPNode(std::string name, RMPNode* parent, int dim):
name(name),
dim(dim),
parent(parent),
x(VectorXd::Zero(dim)),
x_dot(VectorXd::Zero(dim)),
f(VectorXd::Zero(dim)),
M(MatrixXd::Zero(dim, dim))
{
}

RMPNode::~RMPNode(){
    
}

void RMPNode::add_child(RMPNode* node){
    children.push_back(node);
}

void RMPNode::clear_children(){
    this->children.clear();
}

void RMPNode::eval(){
    //this->f = VectorXd::Zero(dim); 
    //this->M = MatrixXd::Zero(dim, dim);   
}



RMPRoot::RMPRoot(std::string _name, int _dim)
:RMPNode(_name, NULL, _dim)
{
}

RMPRoot::~RMPRoot(){

}

VectorXd RMPRoot::psi(const VectorXd& y){
    return VectorXd::Zero(dim);
}

MatrixXd RMPRoot::J(const VectorXd& y){
    return MatrixXd::Zero(dim, dim);
}


MatrixXd RMPRoot::J_dot(const VectorXd& y , const VectorXd& y_dot){
    return MatrixXd::Zero(dim, dim);
}

void RMPRoot::eval(){
    this->f = VectorXd::Zero(dim); 
    this->M = MatrixXd::Zero(dim, dim);
}



RMPLeaf::RMPLeaf(std::string name, RMPNode* parent, int dim)
:RMPNode(name, parent, dim)
{

}

RMPLeaf::~RMPLeaf(){

}

void RMPLeaf::eval(){
    // evaluate GDS
    // c++17
    //auto[f, M] = this->RMP_func(this->x, this->x_dot);
    std::tie(f, M) = this->GDS_func(this->x, this->x_dot);
}


RMPTree::RMPTree(){

}

RMPTree::RMPTree(RMPNode* _root, const std::vector<RMPNode*>& _nodes):
root(_root)
{
    this->root->clear_children();
    
    // set all nodes in proper position
    for(size_t i = 0; i < _nodes.size(); i++){
        this->add_node(_nodes[i]);
    }
#ifdef debug
    std::cout << "children size:" <<root->children.size() << std::endl;
#endif
}

void RMPTree::set_state(RMPNode* node, const VectorXd& _x, const VectorXd& _x_dot){
    if (node == NULL)
        return;
    node->x = _x;
    node->x_dot = _x_dot;

#ifdef debug
std::cout << "x:" << _x << std::endl;
#endif
}

RMPTree::~RMPTree(){

}

void RMPTree::add_node(RMPNode* node){
    if (node == NULL)
        return;
    if (node->parent == NULL)
        return;
    node->parent->add_child(node);
}

void RMPTree::pushforward(RMPNode* node){

/*
pushforward operator handles the state information pass through RMP nodes.  
*/
    if (node == NULL)
        return;

    for(size_t i=0; i < node->children.size(); i++){
        RMPNode* child = node->children[i];
        child->x = child->psi(node->x);
        child->x_dot = child->J(node->x)*node->x_dot;
        pushforward(child);
    }
    
}

void RMPTree::pullback(RMPNode* node){
/*
pullback operator can only be called after pushforward,
cause it needs updated state information.
pullback back propagates the desired force and inertia matrix from leaf nodes to root
*/
    
    // depth first search
    for(size_t i=0; i < node->children.size(); i++)
        pullback(node->children[i]);
    

    // evaluate dynamics
    node->eval();

    for(size_t i=0; i < node->children.size(); i++){
        RMPNode* child = node->children[i];
        MatrixXd J = child->J(node->x);
        MatrixXd J_dot = child->J_dot(node->x, node->x_dot);
        node->f += (J.transpose() * (child->f - child->M*J_dot*node->x_dot));
        
        // inertia matrix
        node->M += (J.transpose() * child->M * J);
        
#ifdef debug
        std::cout << "solved J :\n" << J << std::endl;
        std::cout << "solved J_dot :\n" << J_dot << std::endl;
        
        std::cout << "child f :\n" << child->f << std::endl;
        std::cout << "child M :\n" << child->M << std::endl;

        std::cout << "solved f :\n" << node->f << std::endl;
        std::cout << "solved M :\n" << node->M << std::endl;
#endif
    }
    
}

VectorXd RMPTree::resolve(RMPNode* node){
    return node->M.jacobiSvd(ComputeFullU | ComputeFullV).solve(node->f);
}

VectorXd RMPTree::solve(const VectorXd& x, const VectorXd& x_dot){
    RMPTree::set_state(this->root, x, x_dot);
    pushforward(this->root);
    pullback(this->root);

#ifdef debug
std::cout << "solved acceleration :\n" << a << std::endl;
#endif
    return resolve(this->root);
}

