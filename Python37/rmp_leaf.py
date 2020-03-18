# Leaf node RMP classes
# @author Anqi Li
# @date April 8, 2019

from rmp import RMPNode, RMPLeaf, RMPLeaf_CLF
import numpy as np
from numpy.linalg import norm



# for comparision with Riemannian based method
class NaiveCollisionAvoidance(RMPLeaf):
    def __init__(self, name, parent, obstacle, epsilon=0.2, 
    alpha=1e-5, eta=0):
        R = obstacle.r
        c = obstacle.c

        N = c.shape[0]
        psi = lambda y: np.array(norm(y-c) / R - 1).reshape(-1,1)
        J = lambda y: (y - c).T / (norm(y-c) * R)
        J_dot = lambda y, y_dot: np.dot(y_dot.T, (-1 / norm(y - c) ** 3 * np.dot((y - c), (y - c).T) + 1 / norm(y - c) * np.eye(N))) / R
            
        def RMP_func(x, x_dot):
            w = 1.0 / x**4
            grad_w = -4.0 / x**5
            g = 1
            grad_Phi = alpha* w * grad_w
        
            M = g
            Bx_dot = eta*g*x_dot
        
            f = -grad_Phi - Bx_dot
            f = np.minimum(np.maximum(f, -1e10), 1e10)
            return (f, M)
        RMPLeaf.__init__(self, name, parent, psi, J, J_dot, RMP_func)
    

class CollisionAvoidance_CLF(RMPLeaf_CLF):
    
    def __init__(self, name, parent, obstacle, Alter_func, alpha_func, epsilon=0.2, alpha=1e-5, eta=0):
        R = obstacle.r
        c = obstacle.c
        self.alpha = alpha
        self.epsilon = epsilon
        self.AlterFunc = Alter_func
        self.alphaFunc = alpha_func
        # Jacobi 
    
        if c.ndim == 1:
            c = c.reshape(-1, 1)

        N = c.size

        psi = lambda y: np.array(norm(y - c) / R - 1).reshape(-1,1)
        J = lambda y: 1.0 / norm(y - c) * (y - c).T / R
        J_dot = lambda y, y_dot: np.dot(
            y_dot.T,
            (-1 / norm(y - c) ** 3 * np.dot((y - c), (y - c).T)
                + 1 / norm(y - c) * np.eye(N))) / R
        
        # GDS                        
        def RMP_func(x, x_dot, with_B=True):
            if x < 0:
                w = 1e10
                grad_w = 0
            else:
                w = 1.0 / x ** 4
                grad_w = -4.0 / x ** 5
            
            u = epsilon + np.minimum(0, x_dot) * x_dot
            g = w * u

            grad_u = 2 * np.minimum(0, x_dot)
            grad_Phi = alpha * w * grad_w
            xi = 0.5 * x_dot ** 2 * u * grad_w

            M = g + 0.5 * x_dot * w * grad_u
            M = np.minimum(np.maximum(M, - 1e5), 1e5)

            Bx_dot = eta * g * x_dot
            
            if with_B:
                f = - grad_Phi - xi - Bx_dot
            else:
                f = - grad_Phi - xi
            
            f = np.minimum(np.maximum(f, - 1e10), 1e10)

            return (f, M)
        RMPLeaf_CLF(self, name, parent, psi, J, J_dot, RMP_func, Alter_func, alpha_func)

class CollisionAvoidance(RMPLeaf):
    """
    Obstacle avoidance RMP leaf
    """
    
    def __init__(self, name, parent, obstacle, epsilon=0.2,
        alpha=1e-5, eta=0):

        R = obstacle.r
        c = obstacle.c
        self.alpha = alpha
        self.eta = eta
        self.epsilon = epsilon

        # Jacobi 
    
        if c.ndim == 1:
            c = c.reshape(-1, 1)

        N = c.size

        psi = lambda y: np.array(norm(y - c) / R - 1).reshape(-1, 1)
        J = lambda y: 1.0 / norm(y - c) * (y - c).T / R
        J_dot = lambda y, y_dot: np.dot(
            y_dot.T,
            (-1 / norm(y - c) ** 3 * np.dot((y - c), (y - c).T)
                + 1 / norm(y - c) * np.eye(N))) / R
        
        # GDS                        
        def RMP_func(x, x_dot, terms=False):
            if x < 0:
                w = 1e10
                grad_w = 0
            else:
                w = 1.0 / x ** 4
                grad_w = -4.0 / x ** 5
            
            u = epsilon + np.minimum(0, x_dot) * x_dot
            g = w * u

            grad_u = 2 * np.minimum(0, x_dot)
            grad_Phi = alpha * w * grad_w
            xi = 0.5 * x_dot ** 2 * u * grad_w

            Xi = 0.5 * x_dot * w * grad_u
            M = g + Xi
            M = np.minimum(np.maximum(M, - 1e5), 1e5)

            Bx_dot = eta * g * x_dot
            
            f = - Bx_dot - grad_Phi  - xi
            f = np.minimum(np.maximum(f, - 1e10), 1e10)
            
            if terms:
                return (f, M, g, Xi, xi, Bx_dot, grad_Phi)
            else:
                return (f, M)

        RMPLeaf.__init__(self, name, parent, psi, J, J_dot, RMP_func)



class GoalAttractorUni(RMPLeaf):
    """
    Goal Attractor RMP leaf
    """	

    def __init__(self, name, parent, y_g, w_u=10, w_l=1, sigma=1,
        alpha=1, eta=2, gain=1, tol=0.005):

        if y_g.ndim == 1:
            y_g = y_g.reshape(-1, 1)
        N = y_g.size
        psi = lambda y: (y - y_g)
        
        J = lambda y: np.eye(N)
        J_dot = lambda y, y_dot: np.zeros((N, N))

        def RMP_func(x, x_dot):
            x_norm = norm(x)

            beta = np.exp(- x_norm ** 2 / 2 / (sigma ** 2))
            w = (w_u - w_l) * beta + w_l
            s = (1 - np.exp(-2 * alpha * x_norm)) / (1 + np.exp(
                    -2 * alpha * x_norm))

            G = np.eye(N) * w
            if x_norm > tol:
                grad_Phi = s / x_norm * w * x * gain
            else:
                grad_Phi = 0
            Bx_dot = eta * w * x_dot
            grad_w = - beta * (w_u - w_l) / sigma ** 2 * x

            x_dot_norm = norm(x_dot)
            xi = -0.5 * (x_dot_norm ** 2 * grad_w - 2 *
                np.dot(np.dot(x_dot, x_dot.T), grad_w))

            M = G
            f = - grad_Phi - Bx_dot - xi
            return (f, M)
        
        RMPLeaf.__init__(self, name, parent, psi, J, J_dot, RMP_func)
        #RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)


    def update_goal(self, y_g):
        """
        update the position of the goal
        """
        
        if y_g.ndim == 1:
            y_g = y_g.reshape(-1, 1)
        N = y_g.size
        self.psi = lambda y: (y - y_g)
        self.J = lambda y: np.eye(N)
        self.J_dot = lambda y, y_dot: np.zeros((N, N))

class GoalAttractorUni_CLF(RMPLeaf_CLF):
    """	
    Goal Attractor RMP leaf
    """
    def __init__(self, name, parent, y_g, policy_candidate, alpha_func, w_u=10, w_l=1, sigma=1,
        alpha=1, eta=2, gain=1, tol=0.005):

        if y_g.ndim == 1:
            y_g = y_g.reshape(-1, 1)
        N = y_g.size
        psi = lambda y: (y - y_g)   
        J = lambda y: np.eye(N)
        J_dot = lambda y, y_dot: np.zeros((N, N))
        
        
        def u(x, x_dot):
            x_norm = norm(x)

            beta = np.exp(- x_norm ** 2 / 2 / (sigma ** 2))
            w = (w_u - w_l) * beta + w_l
            s = (1 - np.exp(-2 * alpha * x_norm)) / (1 + np.exp(
                    -2 * alpha * x_norm))
            if x_norm > tol:
                grad_Phi = s / x_norm * w * x * gain
            else:
                grad_Phi = np.zeros_like(x)
            return - grad_Phi
            
            
        def RMP_func(x, x_dot, terms=False):
            x_norm = norm(x)

            beta = np.exp(- x_norm ** 2 / 2 / (sigma ** 2))
            w = (w_u -   w_l) * beta + w_l
            s = (1 - np.exp(-2 * alpha * x_norm)) / (1 + np.exp(
                    -2 * alpha * x_norm))

            G = np.eye(N) * w
            if x_norm > tol:
                grad_Phi = s / x_norm * w * x * gain
            else:
                grad_Phi = 0
            Bx_dot = eta * w * x_dot
            grad_w = - beta * (w_u - w_l) / sigma ** 2 * x

            x_dot_norm = norm(x_dot)
            xi = -0.5 * (x_dot_norm ** 2 * grad_w - 2 *
                np.dot(np.dot(x_dot, x_dot.T), grad_w))

            M = G
            f = - grad_Phi - Bx_dot - xi
            f = np.maximum(np.minimum(f, 1e5), -1e5)

            if terms:
                return (f, M, G, 0, xi, Bx_dot, grad_Phi)
            else:
                return (f, M)
        
        RMPLeaf_CLF.__init__(self, name, parent, psi, J, J_dot, RMP_func, policy_candidate, alpha_func)
        #RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)


    def update_goal(self, y_g):
        """
        update the position of the goal
        """
        
        if y_g.ndim == 1:
            y_g = y_g.reshape(-1, 1)
        N = y_g.size
        self.psi = lambda y: (y - y_g)
        self.J = lambda y: np.eye(N)
        self.J_dot = lambda y, y_dot: np.zeros((N, N))

class CollisionAvoidanceDecentralized(RMPLeaf):
    """
    Decentralized collision avoidance RMP leaf for the RMPForest
    """

    def __init__(self, name, parent, parent_param, c=np.zeros(2), R=1, epsilon=1e-8,
        alpha=1e-5, eta=0):

        assert parent_param is not None

        self.R = R
        self.alpha = alpha
        self.eta = eta
        self.epsilon = epsilon
        self.x_dot_real = None

        psi = None
        J = None
        J_dot = None


        def RMP_func(x, x_dot, x_dot_real):
            if x < 0:
                w = 1e10
                grad_w = 0
            else:
                w = 1.0 / x ** 4
                grad_w = -4.0 / x ** 5
            u = epsilon + np.minimum(0, x_dot) * x_dot
            g = w * u

            grad_u = 2 * np.minimum(0, x_dot)
            grad_Phi = alpha * w * grad_w
            xi = 0.5 * x_dot * x_dot_real * u * grad_w

            M = g + 0.5 * x_dot * w * grad_u
            M = np.minimum(np.maximum(M, - 1e5), 1e5)

            Bx_dot = eta * g * x_dot

            f = - grad_Phi - xi - Bx_dot
            f = np.minimum(np.maximum(f, - 1e10), 1e10)

            return (f, M)


        RMPLeaf.__init__(self, name, parent, parent_param, psi, J, J_dot, RMP_func)


    def pushforward(self):
        """
        override pushforward() to update the curvature term
        """
        if self.verbose:
            print('%s: pushforward' % self.name)

        if self.psi is not None and self.J is not None:
            self.x = self.psi(self.parent.x)
            self.x_dot = np.dot(self.J(self.parent.x), self.parent.x_dot)
            self.x_dot_real = np.dot(
                self.J(self.parent.x),
                self.parent.x_dot - self.parent_param.x_dot)


    def eval_leaf(self):
        """
        override eval_leaf() to update the curvature term
        """
        self.f, self.M = self.RMP_func(self.x, self.x_dot, self.x_dot_real)

    def update_params(self):
        """
        update the position of the other robot
        """
        c = self.parent_param.x
        z_dot = self.parent_param.x_dot
        R = self.R
        if c.ndim == 1:
            c = c.reshape(-1, 1)

        N = c.size

        self.psi = lambda y: np.array(norm(y - c) / R - 1).reshape(-1,1)
        self.J = lambda y: 1.0 / norm(y - c) * (y - c).T / R
        self.J_dot = lambda y, y_dot: np.dot(
            y_dot.T,
            (-1 / norm(y - c) ** 3 * np.dot((y - c), (y - c).T)
                + 1 / norm(y - c) * np.eye(N))) / R


class CollisionAvoidanceCentralized(RMPLeaf):
    """
    Centralized collision avoidance RMP leaf for a pair of robots
    """

    def __init__(self, name, parent, R=1, epsilon=1e-8,
        alpha=1e-5, eta=0):

        self.R = R

        def psi(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.array(norm(y1 - y2) / R - 1).reshape(-1,1)
        def J(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.concatenate((
                    1.0 / norm(y1 - y2) * (y1 - y2).T / R,
                    -1.0 / norm(y1 - y2) * (y1 - y2).T / R),
                axis=1)
        def J_dot(y, y_dot):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            y1_dot = y_dot[: N]
            y2_dot = y_dot[N:]
            return np.concatenate((
                    np.dot(
                        y1_dot.T,
                        (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                            + 1 / norm(y1 - y2) * np.eye(N))) / R,
                    np.dot(
                        y2_dot.T,
                        (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                            + 1 / norm(y1 - y2) * np.eye(N))) / R),
                axis=1)


        def RMP_func(x, x_dot):
            if x < 0:
                w = 1e10
                grad_w = 0
            else:
                w = 1.0 / x ** 4
                grad_w = -4.0 / x ** 5
            u = epsilon + np.minimum(0, x_dot) * x_dot
            g = w * u

            grad_u = 2 * np.minimum(0, x_dot)
            grad_Phi = alpha * w * grad_w
            xi = 0.5 * x_dot ** 2 * u * grad_w

            M = g + 0.5 * x_dot * w * grad_u
            M = np.minimum(np.maximum(M, - 1e5), 1e5)

            Bx_dot = eta * g * x_dot

            f = - grad_Phi - xi - Bx_dot
            f = np.minimum(np.maximum(f, - 1e10), 1e10)

            return (f, M)


        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)





class FormationDecentralized(RMPLeaf):
    """
    Decentralized formation control RMP leaf for the RMPForest
    """

    def __init__(self, name, parent, parent_param, c=np.zeros(2), d=1, gain=1, eta=2, w=1):

        assert parent_param is not None
        self.d = d

        psi = None
        J = None
        J_dot = None


        def RMP_func(x, x_dot):
            G = w
            grad_Phi = gain * x * w
            Bx_dot = eta * w * x_dot
            M = G
            f = - grad_Phi - Bx_dot

            return (f, M)

        RMPLeaf.__init__(self, name, parent, parent_param, psi, J, J_dot, RMP_func)


    def update_params(self):
        """
        update the position of the other robot
        """

        z = self.parent_param.x
        z_dot = self.parent_param.x_dot

        c = z
        d = self.d
        if c.ndim == 1:
            c = c.reshape(-1, 1)

        N = c.size
        self.psi = lambda y: np.array(norm(y - c) - d).reshape(-1,1)
        self.J = lambda y: 1.0 / norm(y - c) * (y - c).T
        self.J_dot = lambda y, y_dot: np.dot(
            y_dot.T,
            (-1 / norm(y - c) ** 3 * np.dot((y - c), (y - c).T)
                + 1 / norm(y - c) * np.eye(N)))



class FormationCentralized(RMPLeaf):
    """
    Centralized formation control RMP leaf for a pair of robots
    """

    def __init__(self, name, parent, d=1, gain=1, eta=2, w=1):

        def psi(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.array(norm(y1 - y2) - d).reshape(-1,1)
        def J(y):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            return np.concatenate((
                    1.0 / norm(y1 - y2) * (y1 - y2).T,
                    -1.0 / norm(y1 - y2) * (y1 - y2).T),
                axis=1)
        def J_dot(y, y_dot):
            N = int(y.size / 2)
            y1 = y[: N]
            y2 = y[N:]
            y1_dot = y_dot[: N]
            y2_dot = y_dot[N:]
            return np.concatenate((
                    np.dot(
                        y1_dot.T,
                        (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                            + 1 / norm(y1 - y2) * np.eye(N))),
                    np.dot(
                        y2_dot.T,
                        (-1 / norm(y1 - y2) ** 3 * np.dot((y1 - y2), (y1 - y2).T)
                            + 1 / norm(y1 - y2) * np.eye(N)))),
                axis=1)


        def RMP_func(x, x_dot):
            G = w
            grad_Phi = gain * x * w
            Bx_dot = eta * w * x_dot
            M = G
            f = - grad_Phi - Bx_dot

            return (f, M)
        RMPLeaf.__init__(self, name, parent, None, psi, J, J_dot, RMP_func)



class Damper(RMPLeaf):
    """
    Damper RMP leaf
    """

    def __init__(self, name, parent, w=1, eta=1):

        psi = lambda y: y
        J = lambda y: np.eye(max(y.shape))
        J_dot = lambda y, y_dot: np.zeros((max(y.shape), max(y.shape)))


        def RMP_func(x, x_dot):
            G = w
            Bx_dot = eta * w * x_dot
            M = G
            f = - Bx_dot

            return (f, M)

        RMPLeaf.__init__(self, name, parent, psi, J, J_dot, RMP_func)

class JointLimit_Avoidance(RMPLeaf):
    """
    JointLimit Avoidance Leaf
    """
    def __init__(self, name, parent, ll, lu, sigma):
        psi = lambda y: y
        J = lambda y: np.eye(max(y.shape))
        J_dot = lambda y, y_dot: np.zeros((max(y.shape), max(y.shape)))
        

        def RMP_func(x, x_dot):
            # diagonal metric matrix
            a = np.zeros(max(x.shape))
            Xi = np.zeros(max(x.shape))
            xi = np.zeros_like(x)

            for i in range(max(x.shape)):
            # how close to upperlimit
                s = (x[i] - ll[i]) / (lu[i] - ll[i])
                d = 4*s*(1-s)
                
                # alpha is the gate indicates if the joint moving towards the limits
                x_dot_p = max(x_dot[i], 0)
                x_dot_n = min(x_dot[i], 0)
                
                alpha_u = 1 - np.exp(-x_dot_p**2/(2*sigma**2))
                alpha_l = 1 - np.exp(-x_dot_n**2/(2*sigma**2))
                
                """
                1. when x -> lu and x_dot > 0
                   b -> d 
                2. when x -> lu and x_dot < 0
                   b -> 1 
                3. when x -> ll and x_dot > 0
                   b -> (1 - s) * 1 -> 1
                4. when x -> ll and x_dot < 0
                   b -> (1 - s) * d -> d
                 
                in other words, if close to either limit and
                moving toward it, use d, otherwise, use 1            
                """
                b = s*(alpha_u*d + (1-alpha_u)*1) + (1 - s)*(alpha_l*d + (1-alpha_l)*1)
                # 1-d matric 
                a[i] = 1 / b**2

                # Xi
                grad_alpha = x_dot[i]*np.exp(-x_dot[i]**2/(2*sigma**2)) / sigma**2
                grad_alphaU = max(grad_alpha, 0)
                grad_alphaL = min(grad_alpha, 0)
                grad_b  = -s*(1-d)*grad_alphaU - (1-s)*(1-d)*grad_alphaL
                Xi[i] = -x_dot[i] * b**(-3) * grad_b

                # xi
                grad_x_b = (alpha_u - alpha_l)*(- 12*s**2 + 8*s - 1) + alpha_l*(1 - 2*s)
                xi[i] = -b**(-3)*x_dot[i]**2*grad_x_b

                # grad_phi
                grad_phi = 0

                # Bx_dot
                Bx_dot = 0
            A = np.diag(a)
            Xi = np.diag(Xi)

            M = A + Xi
            f = -Bx_dot - xi - grad_phi
            return (f, M)
        
        RMPLeaf.__init__(self, name, parent, psi, J, J_dot, RMP_func)
            