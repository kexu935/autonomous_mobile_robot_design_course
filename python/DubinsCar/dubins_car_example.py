import dubins
import math
import matplotlib
import matplotlib.pyplot as plt
import numpy

matplotlib.rcParams['figure.figsize'] = 12, 9

q0 = (0.0, 0.0, math.pi/4)
q1 = (-4.0, 4.0, -math.pi)
turning_radius = 1.0
step_size = 0.5

qs, _ = dubins.path_sample(q0, q1, turning_radius, step_size)
print qs

def expand_axis(ax, scale, name):
    getter = getattr(ax, 'get_' + name)
    setter = getattr(ax, 'set_' + name)
    a, b = getter()
    mid = (a+b)/2.0
    diff = (b - mid)
    setter(mid - scale*diff, mid + scale*diff)

def expand_plot(ax, scale = 1.1):
    expand_axis(ax, scale, 'xlim')
    expand_axis(ax, scale, 'ylim')

def plot_dubins_path(q0, q1, r=turning_radius, step_size=step_size):
    qs, _ = dubins.path_sample(q0, q1, r, step_size)
    qs = numpy.array(qs)
    xs = qs[:, 0]
    ys = qs[:, 1]
    us = xs + numpy.cos(qs[:, 2])
    vs = ys + numpy.sin(qs[:, 2])
    plt.plot(xs, ys, 'b-')
    plt.plot(xs, ys, 'r.')
    for i in xrange(qs.shape[0]):
        plt.plot([xs[i], us[i]], [ys[i], vs[i]],'r-')
    ax = plt.gca()
    expand_plot(ax)
    ax.set_aspect('equal')
    plt.show()

if __name__ == "__main__":
    plot_dubins_path(q0, q1)
