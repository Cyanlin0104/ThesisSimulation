# RMPflow basic classes
# @author Anqi Li
# @date April 8, 2019

import numpy as np

class RMPTree:
    
    def __init__(self, root, nodes=None, ignoreCurvature=False, nullspace=False):
        # add all nodes to the tree
        self.root = root
        self.ignoreCurvature = ignoreCurvature
        self.nullspace = nullspace
        self.root.clear()

        [self.add_node(node) for node in nodes]
        
    
    def add_node(self, node):
        if node is not None:
            if node.parent is not None:
                node.parent.add_child(node)
    
    
    def set_state(self, node, x, x_dot):
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
    
    
    def eval_GDS(self, node):
        if node.RMP_func is not None:
            node.f, node.M = node.RMP_func(node.x, node.x_dot)
    
    
            
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
        
        if node.RMP_func is not None:
            self.eval_GDS(node)
        else:
            node.f = np.zeros_like(node.x, dtype='float64')
            node.M = np.zeros([max(node.x.shape), max(node.x.shape)], dtype='float64')
        
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
        if node.RMP_func is not None:
            self.eval_GDS(node)
        # root node may has not designed geometry imposed 
        else: 
            node.f = np.zeros_like(node.x, dtype='float64')
            node.M = np.zeros([max(node.x.shape), max(node.x.shape)], dtype='float64')
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
            
    def resolve(self, node):
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
        else:
            self.pullback(self.root, self.ignoreCurvature)
        
        return self.resolve(self.root)


class RMPNode:
    """
    A Generic RMP node
    """
    def __init__(self, name, parent, psi, J, J_dot, RMP_func, verbose=False):
        self.name = name

        self.parent = parent
        self.children = []
        self.RMP_func = RMP_func
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
    
    def clear(self):
        self.children.clear()

class RMPRoot(RMPNode):
    """
    A root node
    """

    def __init__(self, name):
        RMPNode.__init__(self, name, None, None, None, None, None)

class RMPLeaf(RMPNode):
    """
    A leaf node
    """

    def __init__(self, name, parent, parent_param, psi, J, J_dot, RMP_func):
        RMPNode.__init__(self, name, parent, psi, J, J_dot)
        self.RMP_func = RMP_func
        self.parent_param = parent_param

if __name__ == '__main__':
    from rmp import RMPNode
    from rmp_leaf import NaiveCollisionAvoidance, CollisionAvoidance, GoalAttractorUni, Damper
    import numpy as np
    from numpy.linalg import norm
    from scipy.integrate import odeint
    import matplotlib.pyplot as plt
    from rmp_util import obstacle, scene, policy_evaluator, policy_evaluator2
    
    x_g = np.array([-1, 2.6])
    x_o = np.array([0, 0])
    x_o2 = np.array([0, 2])
    x = np.array([2.5, -3.2])
    x_dot = np.array([-1, 2])
    
    test_scene = scene(goal=x_g, obstacle=[obstacle(x_o, 0.5)], x=x, x_dot=x_dot)
    
    r = RMPNode('Root', None, None, None, None, None, None)
    #def __init__(self, name, parent, psi, J, J_dot, RMP_func, verbose=False):
    #leaf1 = CollisionAvoidance('collision_av
    #oidance', r, None, 
    #                           epsilon=0.2, alpha=0.01, c=test_scene.obstacle[0].c, 
    #                           R=test_scene.obstacle[0].r)
    leaf2 = GoalAttractorUni('goal_attractor', r, test_scene.goal)
    tree = RMPTree(root=r, nodes=[leaf2])
    tspan = np.linspace(0,1,2)
    pe = policy_evaluator2(test_scene, tree, tspan)
    
    