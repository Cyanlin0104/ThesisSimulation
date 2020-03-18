# RMPflow basic classes
# @author Cyanlin
# @date Mar 8, 2020

import numpy as np

# if do not use CLF method, one can uncomment this line
from cvxopt import matrix, solvers


class RMPTree:
    
    def __init__(self, root, nodes=None, ignoreCurvature=False, nullspace=False, lite=False):
        # add all nodes to the tree
        self.root = root
        self.ignoreCurvature = ignoreCurvature
        self.nullspace = nullspace
        self.lite = lite
        self.root.clear_children()

        [self.add_node(node) for node in nodes]
        
    
    def add_node(self, node):
        if node is not None:
            if node.parent is not None:
                node.parent.add_child(node)
    
    @staticmethod
    def set_state(node, x, x_dot):
        if node is None:
            return
        
        assert x.ndim == 1 or x.ndim == 2
        assert x_dot.ndim == 1 or x_dot.ndim == 2

        if x.ndim == 1:
            x = x.reshape(-1, 1)
        if x_dot.ndim == 1:
            x_dot = x_dot.reshape(-1, 1)
        node.x = x
        node.x_dot = x_dot

    def pushforward(self, node):
        """
        Args:
            inputs:
             node: the node with which the operation start
            
            outputs:
             None
        """
        if node is None:
            print('node cannot be None!')
            return
            
        for child in node.children:
            child.x = child.psi(node.x)
            child.x_dot = np.dot(child.J(node.x), node.x_dot)
            self.pushforward(child)
    
    def pullback_nullspace(self, node):
        [self.pullback(child) for child in node.children]
        
        node.eval()
        
        # 1-th node is the primary task
        pri = node.children[0]
        sec = node.children[1]
        J1 = pri.J(node.x)
        J1_dot = pri.J_dot(node.x, node.x_dot)
        J2 = sec.J(node.x)
        J2_dot = sec.J_dot(node.x, node.x_dot)
        
        # f = J1.T * f1_ + N1 * fn
        # 
        # where f1_ = f1 - M1*J1_dot*x_dot
        #       f2_ = f2 - M2*J2_dot*x_dot
        #
        #.      fn = J2.T * f2_
        #.      N1 = (I - J1.T * pinv(J1.T))
        
        f1_ = pri.f - np.dot(pri.M, np.dot(J1_dot,node.x_dot))
        
        f2_ = sec.f - np.dot(sec.M, np.dot(J2_dot, node.x_dot))
        
        N1 = (np.eye(J1.shape[1]) - np.dot(np.linalg.pinv(J1), J1))
        
        fn = np.dot(J2.T, f2_)
        node.f += np.dot(J1.T, f1_) + np.dot(N1, fn)
        
        node.M += np.dot(np.dot(J1.T, pri.M), J1)
        node.M += np.dot(np.dot(J2.T, sec.M), J2)
        
    def pullback(self, node, ignoreCurvature=False):
        """
        Args:
            inputs:
             node: the node with which the operation start
            
            outputs:
             None
        `"""
        [self.pullback(child) for child in node.children]
        # for each task space,first evaluate designed geometry dynamics
        node.eval()
        # pullback all  
        for child in node.children:
            J = child.J(node.x)
            J_dot = child.J_dot(node.x, node.x_dot)
            assert J.ndim == 2 and J_dot.ndim == 2
            
            if(ignoreCurvature):
                node.f += np.dot(J.T, child.f)
            else:
                node.f += np.dot(J.T, (child.f - np.dot(child.M, np.dot(J_dot, node.x_dot))))
            
            #node.M = np.eye(node.x.shape[0])
            node.M += np.dot(np.dot(J.T, child.M), J)
    
    @staticmethod
    def resolve(node):
        """
        Args:
            inputs:
             node: the node to be resolved
            
            outputs:
             a: the acceleration resolved from force and inertia matrix 
        """
        #print("root node M \n", node.M)
        #print("root node f \n", node.f)
        return np.dot(np.linalg.pinv(node.M), node.f)
    

    def solve(self, x, x_dot):
        self.set_state(self.root, x, x_dot)
        self.pushforward(self.root)
        if self.nullspace:
            self.pullback_nullspace(self.root)
        elif self.lite:
            self.relative_pullback(self.root)
        else:
            self.pullback(self.root, self.ignoreCurvature)
        return self.resolve(self.root)
    
    def relative_pullback(self, root):
        # In this version the two leafs must share one parent
        if len(root.children) != 2:
            raise ValueError('lite only support two child nodes')
        node = root.children[0]
        node_rel = root.children[1]
        # evaluate relative rmp
        node_rel.f, node_rel.M, g, Xi, xi, Bx_dot, grad_Phi = node_rel.RNP_func(node_rel.x, node_rel.x_dot, terms=True)
        x = root.x
        x_dot = root.x_dot
        Jg = node.J(x)
        Jg_dot = node.J_dot(x, x_dot)
        Jc = node_rel.J(x)
        # this is the reason why one should first eval node_rel
        Mc = node_rel.M
        Jg_pinv = np.linalg.pinv(Jg)
        #Mg = np.dot(Jc.T, np.dot(Mc, Jc))
        Mg = Mc*np.eye(max(node.x.shape))
        
        # evaluate major rmp
        node.eval()
        # composed inertia matrix
        # ToDo add curvature?
        node.M += Mg 

        # pullback major RMP to root
        root.eval()
        root.f += np.dot(Jg.T, (node.f - np.dot(node.M, np.dot(Jg_dot, root.x_dot))))
        root.M += np.dot(np.dot(Jg.T, node.M), Jg)


class RMPNode:

    """
    A Generic RMP node
    """
    def __init__(self, name, parent, psi, J, J_dot, verbose=False):
        self.name = name

        self.parent = parent
        self.children = []
        # connect the node to its parent
        #if self.parent:
        #    self.parent.add_child(self)

        # mapping/J/J_dot for the edge from the parent to the node
        self.psi = psi
        self.J = J
        self.J_dot = J_dot

        # state
        self.x = None
        self.x_dot = None

        # RMP
        self.f = None
        self.a = None
        self.M = None

        # print the name of the node when applying operations if true
        self.verbose = verbose


    def add_child(self, child):
        """
        Add a child to the current node
        """
        self.children.append(child)
    
    def clear_children(self):
        self.children.clear()        
    
    def eval(self):
        self.f = np.zeros_like(self.x, dtype='float64')
        self.M = np.zeros([max(self.x.shape), max(self.x.shape)], dtype='float64') 
               
        
class RMPRoot(RMPNode):
    """
    A root node
        def __init__(self, name, parent, psi, J, J_dot, verbose=False):
    """

    def __init__(self, name):
        RMPNode.__init__(self, name, None, None, None, None, None)
    


class RMPLeaf(RMPNode):
    """
    A leaf node
    """

    def __init__(self, name, parent, psi, J, J_dot, RMP_func):
        RMPNode.__init__(self, name, parent, psi, J, J_dot)
        self.RMP_func = RMP_func
    
    def eval(self):
        if self.RMP_func is None:
            return
        self.f, self.M = self.RMP_func(self.x, self.x_dot)

class RMPLeaf_CLF(RMPLeaf):
    """
    Leaf node with Control Lyapunov Function constriant
    """
    
    def __init__(self, name, parent, psi, J, J_dot, RMP_func, policy_candidate, alpha_func):
        RMPLeaf.__init__(self, name, parent, psi, J, J_dot, RMP_func)
        
        
        self.policy_candidate = policy_candidate
        self.alpha_func = alpha_func
        self.V_dot = [] #for debug
        
    def eval(self):
        if self.RMP_func is None:
            return 
        _, self.M, _, _, xi, _, grad_Phi = self.RMP_func(self.x, self.x_dot, terms=True)
        
        # for inequality constraint Gx <= h
        h = np.dot(self.x_dot.T, -xi - grad_Phi) - self.alpha_func(self.x_dot)
        G = self.x_dot.T 
        
        # and cost function
        P = np.eye(max(self.x.shape))
        q = -2*np.dot(self.policy_candidate(self.x, self.x_dot).T, self.M).T
        

        # fit optimizer 
        P = matrix(P)
        q= matrix(q)
        G = matrix(G)
        h = matrix(h)
        
        solvers.options['show_progress'] = False
        sol = solvers.qp(P, q, G, h)
        
        
        # optimal
        self.f = np.array(sol['x'])

        self.V_dot.append(np.dot(self.x_dot.T, self.f + xi + grad_Phi))
        #print(sol['status'])
        #print(V_dot)
        
    