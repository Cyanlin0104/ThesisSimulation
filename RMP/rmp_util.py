from rmp import RMPRoot
from rmp_leaf import NaiveCollisionAvoidance, CollisionAvoidance, GoalAttractorUni, Damper
import numpy as np
from numpy.linalg import norm
from scipy.integrate import odeint
import matplotlib.pyplot as plt

from matplotlib import animation, rc


class obstacle:
    def __init__(self, c, r=1):
        self.r = r
        self.c = c.reshape(-1,1)

class scene:
    def __init__(self, goal=None, obstacle=None, x=None, x_dot=None):
        self.goal = goal
        self.obstacle = obstacle
        if goal is not None:
            goal.reshape(-1, 1)
            self.dim = goal.size
        if obstacle is not None:
            self.num_obs = len(obstacle)
            [obs.c.reshape(-1,1) for obs in obstacle]
            self.dim = obstacle[0].c.size
        if x is None:
            x = np.zeros(dim)
        if x_dot is None:
            x_dot = np.zeros(dim)
        self.init_state = np.concatenate((x, x_dot), axis=None)

    def set_init_state(x, x_dot):
        self.init_state = np.concatenate((x, x_dot), axis=None)
    
    
class policy_evaluator:
    def __init__(self, scene, r, leafs, tspan):
        self.scene = scene
        self.r = r
        self.tspan = tspan
        [r.add_child(leaf) for leaf in leafs]
        #self.sol = ode(self.dynamics, self.tspan, self.scene.init_state)
        self.sol = odeint(self.dynamics, self.scene.init_state, self.tspan)               
    
    def dynamics(self, state, t):
        state = state.reshape(2, -1)
        x = state[0]
        x_dot = state[1]
        x_ddot = self.r.solve(x, x_dot)
        state_dot = np.concatenate((x_dot, x_ddot), axis=None)
        return state_dot
    
    def step_cost(self):
        for i in range(self.sol.shape[0]):
            if( norm(self.sol[i][:2] - self.scene.goal) < 1e-2):
                #print('step cost: {0}'.format(i))
                return (i, self.tspan[i])
        print('task failed, goal has not achieved')
        return None
        
    def plot(self, fig, axes):
        if self.scene.dim != 2:
            return
        axes.plot(self.sol[:, 0], self.sol[:, 1])
        if self.scene.goal is not None:
            axes.plot(self.scene.goal[0], self.scene.goal[1], 'go')
        for obs in self.scene.obstacle:
            circle = plt.Circle((obs.c[0], obs.c[1]), obs.r, color='k', fill=False)
            axes.add_artist(circle)
        
        axes.axis([-5,5,-5,5])
        axes.set_aspect('equal', 'box')
    
    def video(self, fig, axes, frames=None, interval=None):
        if frames is None:
            frames = self.step_cost()[0]
        if interval is None:
            interval = 20
        x, y = [], []
        ln, = plt.plot(x, y, 'b-')        
        def init():
            if self.scene.goal is not None:
                axes.plot(self.scene.goal[0], self.scene.goal[1], 'go')
            for obs in self.scene.obstacle:
                circle = plt.Circle((obs.c[0], obs.c[1]), obs.r, color='k', fill=False)
                axes.add_artist(circle)
            axes.axis([-5,5,-5,5])
            return ln,
        def update(i):
            x.append(self.sol[i][0])
            y.append(self.sol[i][1])
            ln.set_data(x, y)
            return ln,
        return animation.FuncAnimation(fig, update, frames=frames, interval=interval, init_func=init, blit=True)
        
# --------------------------------------------
class policy_evaluator2:
    def __init__(self, scene, tree, tspan):
        self.scene = scene
        self.tree = tree
        self.tspan = tspan
        self.sol = odeint(self.dynamics, self.scene.init_state, self.tspan)               
    
    def dynamics(self, state, t):
        state = state.reshape(2, -1)
        x = state[0]
        x_dot = state[1]
        x_ddot = self.tree.solve(x, x_dot)
        state_dot = np.concatenate((x_dot, x_ddot), axis=None)
        return state_dot
    
    def step_cost(self):
        for i in range(self.sol.shape[0]):
            if( norm(self.sol[i][:2] - self.scene.goal) < 1e-2):
                return (i, self.tspan[i])
        print('task failed, goal has not achieved')
        return None
        
    def plot(self, fig, axes):
        if self.scene.dim != 2:
            return
        axes.plot(self.sol[:, 0], self.sol[:, 1])
        if self.scene.goal is not None:
            axes.plot(self.scene.goal[0], self.scene.goal[1], 'go')
        for obs in self.scene.obstacle:
            circle = plt.Circle((obs.c[0], obs.c[1]), obs.r, color='k', fill=False)
            axes.add_artist(circle)
        
        axes.axis([-5,5,-5,5])
        axes.set_aspect('equal', 'box')
    
    def video(self, fig, axes, frames=None, interval=None):
        if frames is None:
            frames = self.step_cost()[0]
        if interval is None:
            interval = 20
        x, y = [], []
        ln, = plt.plot(x, y, 'b-')        
        def init():
            if self.scene.goal is not None:
                axes.plot(self.scene.goal[0], self.scene.goal[1], 'go')
            for obs in self.scene.obstacle:
                circle = plt.Circle((obs.c[0], obs.c[1]), obs.r, color='k', fill=False)
                axes.add_artist(circle)
            axes.axis([-5,5,-5,5])
            return ln,
        def update(i):
            x.append(self.sol[i][0])
            y.append(self.sol[i][1])
            ln.set_data(x, y)
            return ln,
        ani = animation.FuncAnimation(fig, update, frames=frames, interval=interval, init_func=init, blit=True)
        return ani
        
# --------------------------------------------