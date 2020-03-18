import numpy as np
def ode_Euler(f, tspan, x0):
    """
    Args:
     
     inputs:
        f: A function : x -> x_dot
        x0: init value of x
        tspan: a n-d array, from t0 to t_end
     
     outputs:
        x: a n-d array: [x0, x1, x2, ... , xn]
        
    """
    if(type(tspan) is list):
    	tspan = np.linspace(tspan[0], tspan[1], 1000)
    n = tspan.size
    delta_t = (tspan[-1] - tspan[0])/n
    xs = np.zeros([n, len(x0)])
    xs[0] = x0
    for i in range(1, n):
        xs[i] = xs[i-1] + np.array(f(0, xs[i-1]))*delta_t
    return xs


def ode_RK(f, x0, tspan):
    """
    Args:
     
     inputs:
        f: A function : x -> x_dot
        x0: init value of x
        tspan: a n-d array, from t0 to t_end
     
     outputs:
        x: a n-d array: [x0, x1, x2, ... , xn]
        
    """
    if(type(tspan) is list):
    	tspan = np.linspace(tspan[0], tspan[1], 1000)
    n = tspan.size
    h = (tspan[-1] - tspan[0])/n
    xs = np.zeros([n, len(x0)])
    xs[0] = x0
    alpha = 2/3
    Beta2 = 1 / (2*alpha)
    Beta1 = 1 - Beta2
    for i in range(1, n):
        k1 = f(xs[i-1], 0)
        k2 = f(xs[i-1] + alpha*h*k1, 0)
        xs[i] = xs[i-1] + (Beta1*k1 + Beta2*k2)*h
    return xs
