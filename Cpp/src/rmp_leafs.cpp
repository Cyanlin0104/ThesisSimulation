#include "../Include/rmp_leafs.h"


/* Notes Eigen

pass eigen matrix as parameter:
void my_function(Eigen::Vector2d v);
needs to be rewritten as follows, passing v by reference:

void my_function(const Eigen::Vector2d& v);


* Likewise if you have a class having a Eigen object as member:
struct Foo
{
  Eigen::Vector2d v;
};
void my_function(Foo v);

* This function also needs to be rewritten like this:
void my_function(const Foo& v);

* A typical problem is when returning an expression involving temporary matrices created in the function itself, like:

Transpose<MatrixXd> foo(const MatrixBase<Derived> &m) {
  MatrixXd tmp = 2*m;
  return tmp.transpose();
}

because the returned Transpose expression stores a reference to tmp which is dead. Therefore, you should not use expression as a return value;


 --- 

Eigen::MatrixXf load_from_gpu()
{
    Eigen::MatrixXf mat(m_rows,m_cols);
    clEnqueueReadBuffer(m_manager->m_gpu_queue_loader, m_buffer, CL_TRUE, 0, sizeof(float)*numel(), mat.data(), 0, NULL, NULL); 
    return mat; 
}

*Your compiler should be able to do this for you, using the common Return Value Optimization method. Basically what this does, is that the compiler rewrites load_from_gpu to take a pointer to an Eigen::MatrixXf as a parameter, and fill that matrix directly.



Note that it can only do this because it can see that mat will always be the return value, if there are several matrices in the methods and one gets returned based on some condition, the compiler doesn't know which one to replace with the hidden pointer parameter. In this case you have to resort to doing it manually, like in alrikai's answer.

To enable the optimization you have to compile with -O2 with GCC.


*/


CollisionAvoidance::CollisionAvoidance(std::string name, RMPNode* parent, const obstacle& obs, double epsilon, double alpha, double eta)
:RMPLeaf(name, parent, 1),
obs(obs),
epsilon(epsilon),
alpha(alpha),
eta(eta)
{
    //if(obs.x.size() != parent->get_dim())
        //throw "Colliosion Avoidance RMP: dimension error: \n obstacle dimension should be the same as parent RMP dimension";
}


CollisionAvoidance::~CollisionAvoidance(){

}

VectorXd CollisionAvoidance::psi(const VectorXd& y){
    VectorXd x_(1);
    x_ << (y - obs.x).norm() / obs.r - 1;
    return x_;
}

MatrixXd CollisionAvoidance::J(const VectorXd& y){
    MatrixXd J_ = 1.0 / (y - obs.x).norm() * (y - obs.x).transpose() / obs.r;
    return J_;
}

MatrixXd CollisionAvoidance::J_dot(const VectorXd& y, const VectorXd& y_dot){
    int d = obs.x.size();
    VectorXd dis = y - obs.x;
    MatrixXd J_dot_ = (y_dot.transpose() * (-1.0 / pow(dis.norm(),3) * (dis*dis.transpose()) + 1.0 / dis.norm() * MatrixXd::Identity(d,d))) / obs.r;
    return J_dot_;
}


std::tuple<VectorXd, MatrixXd> CollisionAvoidance::GDS_func (const VectorXd& _x , const VectorXd& _x_dot){

    // 1-dimension
    double x = _x[0];
    double x_dot = _x_dot[0];

    double w;
    double grad_w;

    // w(x)
    if(x < 0){
        w = 1e10;
        grad_w = 0;
    }
    else
    {
        w = 1.0 / pow(x, 4);
        grad_w = -4.0 / pow(x, 5);
    }
    // u(x_dot)
    double u = epsilon + std::min(0.0, x_dot) * x_dot;
    
    // g(x, x_dot) = w(x) * u(x_dot)
    double g = w * u;
    double grad_u = 2 * std::min(0.0, x_dot);
    
    double grad_Phi = alpha * w * grad_w;
    double xi = 0.5 * pow(x_dot, 2) * grad_w * u;
    double Xi = 0.5 * x_dot * grad_u * w;
    
    double Bx_dot = eta * g * x_dot;

    double m = g + Xi;
    MatrixXd M(1, 1);
    m = std::min(m, 1e5);
    M << m;
    
    VectorXd f(1);
    f << std::max(std::min(-Bx_dot - grad_Phi - xi, 1e5), -1e5);

    return std::tie(f, M);
}


GoalAttractorUni::GoalAttractorUni(std::string name , RMPNode* parent, const goal& _g, double w_u, double w_l, double sigma, double alpha, double eta, double gain, double tol)
:RMPLeaf(name, parent, _g.get_dim()),
w_u(w_u),
w_l(w_l),
sigma(sigma),
alpha(alpha),
eta(eta),
gain(gain),
tol(tol),
g(_g)
{
}

GoalAttractorUni::~GoalAttractorUni(){
    
}

VectorXd GoalAttractorUni::psi(const VectorXd& y){
    VectorXd x_ = y - g.get_x();
    return x_;
}

MatrixXd GoalAttractorUni::J(const VectorXd& y){
    MatrixXd J_ = MatrixXd::Identity(parent->get_dim(), dim);
    return J_;
}

MatrixXd GoalAttractorUni::J_dot(const VectorXd& y, const VectorXd& y_dot){
    MatrixXd J_dot_ = MatrixXd::Zero(parent->get_dim(), dim);
    return J_dot_;
}


std::tuple<VectorXd, MatrixXd> GoalAttractorUni::GDS_func (const VectorXd& x , const VectorXd& x_dot){
    double x_norm = x.norm();
    double beta = exp(-x_norm*x_norm / 2 / (sigma*sigma));
    double w = (w_u - w_l) * beta + w_l;
    double s = ( 1 - exp(-2*alpha*x_norm) / (1 + exp(-2 * alpha * x_norm)));

    // G = I * w
    VectorXd grad_Phi;
    if(x_norm > tol)
        grad_Phi = (s / x_norm * w * gain * x);
    else
        grad_Phi = VectorXd::Zero(dim);

    VectorXd Bx_dot = eta * w * x_dot;
    VectorXd grad_w = -beta*(w_u - w_l) / (sigma*sigma) * x;

    double x_dot_norm = x_dot.norm();
    VectorXd xi = -0.5 * (x_dot_norm*x_dot_norm * 2 * grad_w - 2 * x_dot * x_dot.transpose() * grad_w);

    MatrixXd M = MatrixXd::Identity(dim, dim) * w;
    VectorXd f = -grad_Phi - Bx_dot - xi;

    // std::cout << "f, M, xi, Bxdot \n" << f << std::endl 
    // << M  << std::endl  << xi << std::endl  << Bx_dot << std::endl; 
    return std::tie(f, M);

}