from .task_interface import TaskInterface
from ..path_planning.romi.romi.groups import *
from ..path_planning.romi.romi.movement_primitives import *
from ..path_planning.romi.romi.trajectory import *


class Reacher2D(TaskInterface):

    def __init__(self,  n_features, points=0, headless=True):

        super().__init__()
        self._group = Group("reacher2d", ["j%d" % i for i in range(2)])
        self._space = ClassicSpace(self._group, n_features)

        self._state_dim = 2
        self._headless = headless

        self._n_points = points
        self._goals = [self._point(3/2, np.pi/8),
                       self._point(1., np.pi/2 + np.pi/8),
                       self._point(2/3, np.pi + np.pi/4),
                       self._point(1/2, 3/2*np.pi + np.pi/6)]
        self._kinematics = Forward2DKinematics(1., 1.)
    
        self._context = None

    def _point(self, d, theta):
        return d*np.array([np.cos(theta), np.sin(theta)])

    def _generate_context(self, goal=None):
        if self._n_points == 0:
            d = np.random.uniform(0, 1)
            a = np.random.uniform(-np.pi, np.pi)
            return self._point(d, a)
        else:
            if goal is None:
                k = np.random.choice(range(self._n_points))
            else:
                k = goal
            g = self._goals[k]
            d = np.random.uniform(0, 1/5)
            a = np.random.uniform(-np.pi, np.pi)
            return g + self._point(d, a)

    def give_example(self, goal=None):
        goal = self._generate_context(goal)
        conf, traj = self._kinematics.get_trajectory(0., 0., goal)
        return goal, conf, traj

    def _generate_demo(self):
        goal = self._generate_context()
        conf, traj = self._kinematics.get_trajectory(0., 0., goal)
        trajectory = NamedTrajectory(*self._group.refs)
        for c in conf:
            trajectory.notify(duration=1/100.,
                              j0=c[0], j1=c[1])
        return goal, np.array([3.]), LearnTrajectory(self._space, trajectory).get_block_params()

    def get_context_dim(self):
        return self._state_dim

    def read_context(self):
        return self._context

    def get_demonstrations(self):
        return np.array([np.concatenate(self._generate_demo(), axis=0) for _ in range(100)])

    def send_movement(self, weights, duration):
        mp = MovementPrimitive(self._space, MovementPrimitive.get_params_from_block(self._space, weights))
        duration = 1 if duration < 0 else duration
        trajectory = mp.get_full_trajectory(duration=duration, frequency=200)
        
        vals = trajectory.get_dict_values()

        reward = -self._kinematics.get_loss(vals["j0"][-1], vals["j1"][-1], self._context)

        return reward, reward
    
    def get_movement(self, weights, duration):
        mp = MovementPrimitive(self._space, MovementPrimitive.get_params_from_block(self._space, weights))
        duration = 1 if duration < 0 else duration
        trajectory = mp.get_full_trajectory(duration=duration, frequency=200)
    
        vals = trajectory.get_dict_values()
        
        return vals

    def reset(self):
        self._context = self._generate_context()
        
        
class Forward2DKinematics:
    """ for our 2D robot """
    def __init__(self, d1, d2):
        self._d1 = d1
        self._d2 = d2

    def _link(self, d):
        return np.array([d, 0.])

    def _rot(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])

    def get_forward(self, theta1, theta2):
        x1 = self._rot(theta1) @ self._link(self._d1)
        r1 = self._rot(theta1) @ self._rot(0.)
        r2 = self._rot(theta2) @ r1
        x2 = r2 @ self._link(self._d2) + x1
        return x2

    def get_full_forward(self, theta1, theta2):
        x1 = self._rot(theta1) @ self._link(self._d1)
        r1 = self._rot(theta1) @ self._rot(0.)
        r2 = self._rot(theta2) @ r1
        x2 = r2 @ self._link(self._d2) + x1
        return x1, x2

    def get_loss(self, theta1, theta2, goal):
        ref = self.get_forward(theta1, theta2)
        return np.mean((ref - goal)**2)

    def jac(self, theta1, theta2, goal, delta=1E-5):
        ref = self.get_loss(theta1, theta2, goal)
        j1 = (self.get_loss(theta1 + delta, theta2, goal) - ref)/delta
        j2 = (self.get_loss(theta1, theta2+delta, goal) - ref)/delta
        return np.array([j1, j2])

    def get_trajectory(self, theta1, theta2, goal, v=0.1):
        conf = [np.array([theta1, theta2])]
        for _ in range(200):
            conf.append(conf[-1]-v*self.jac(conf[-1][0], conf[-1][1], goal))
        return conf, [self.get_forward(c[0], c[1]) for c in conf]
