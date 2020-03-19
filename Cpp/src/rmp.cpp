#include "../Include/rmp.h"

RMPNode::RMPNode(){

}

RMPNode::RMPNode(std::string _name, 
                RMPNode* _parent, 
                int _dim, 
                VectorXd (*_psi)(const VectorXd&), 
                MatrixXd (*_J)(const VectorXd&), 
                MatrixXd (*_J_dot)(const VectorXd&, const VectorXd&)):
name(_name),
dim(_dim),
parent(_parent)
{
    this->psi = _psi;
    this->J = _J;
    this->J_dot = _J_dot;
    this->x = VectorXd::Zero(dim);
    this->x_dot = VectorXd::Zero(dim);
    this->f = VectorXd::Zero(dim);
    this->M = MatrixXd::Zero(dim, dim);
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

}



RMPRoot::RMPRoot()
:RMPNode()
{

}

RMPRoot::RMPRoot(std::string _name, int _dim)
:RMPNode(_name, NULL, _dim, NULL, NULL, NULL)
{
}


void RMPRoot::eval(){
    this->f = VectorXd::Zero(dim); 
    this->M = MatrixXd::Zero(dim, dim);   
}

RMPLeaf::RMPLeaf(){

}

RMPLeaf::RMPLeaf(std::string _name, 
                RMPNode* _parent, 
                int _dim, 
                VectorXd (*_psi)(const VectorXd&), 
                MatrixXd (*_J)(const VectorXd&), 
                MatrixXd (*_J_dot)(const VectorXd&, const VectorXd&),
                std::tuple<VectorXd, MatrixXd> (*_RMP_func)(VectorXd, VectorXd))
: RMPNode(_name, _parent, _dim, _psi, _J, _J_dot)
{
    this->RMP_func = _RMP_func;
}

void RMPLeaf::eval(){
    if(this->RMP_func == NULL)
        return;
    auto[f, M] = this->RMP_func(this->x, this->x_dot);
    
}



RMPTree::RMPTree(){

}

RMPTree::RMPTree(RMPNode* _root, std::vector<RMPNode*> _nodes):
root(_root)
{
    this->root->clear_children();
    // set all nodes in proper position
    for(size_t i = 0; i < _nodes.size(); i++){
        this->add_node(_nodes[i]);
    }
}

void RMPTree::set_state(RMPNode* node, VectorXd _x, VectorXd _x_dot){
    if (node == NULL)
        return;
    
    node->x = _x;
    node->x_dot = _x_dot;
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

    for(size_t i=0; node->children.size(); i++){
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
        
        // desired force 
        node->f += J.transpose() * (child->f - child->M*J_dot*node->x_dot);
        // inertia matrix
        node->M += J.transpose() * child->M * J
    }
    
}

void RMPTree::resolve(RMPNode* node){

}

void RMPTree::solve(RMPNode* node){

}



int main(){
    RMPRoot* r = new RMPRoot("r", 1);
    RMPLeaf* leaf = new RMPLeaf("leaf", r, 1, NULL, NULL, NULL, NULL);
    
    VectorXd x;
    VectorXd b(3);
    b << 1, 2, 3;
    x = b;
    
    std::cout << x << std::endl;
    delete r;
    delete leaf;
    
    return 0;
}
