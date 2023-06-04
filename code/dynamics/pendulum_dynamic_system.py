import numpy as np

from graphics import *
from dynamics import *
from geom import *
from .abstract_dynamic_system import AbstractDynamicSystem

## Pendulum dynamic system just to test
class PendulumDynamicSystem(AbstractDynamicSystem):

    def __init__(self, mesh, symplectic):
        ## Constructor
        # @param self
        # @param mesh  
        super().__init__()
        self.mesh = mesh
        self.symplectic = symplectic

        # Animations parameters
        self.delta = 0.02
        self.g = 9.81
        self.l = 1.
        self.m = 1.
        self.theta = np.pi/5.
        self.theta_speed = 0.

    def step(self):
        self.old_angle = self.theta
        self.theta += self.delta * self.theta_speed
        self.theta_speed += self.delta * (-self.g/self.l) * np.sin(self.theta*(self.symplectic) + self.old_angle*(not self.symplectic))
        x = self.l * np.sin(self.theta)
        y = - self.l * np.cos(self.theta)
        self.mesh.positions = np.array([0., 0., x, y], np.float64)
        T = 0.5*self.l*self.theta_speed*self.theta_speed
        U = self.m * self.g * y
        Em = T + U
        print(abs(Em))



