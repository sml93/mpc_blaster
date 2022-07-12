import sys
sys.path.insert(0, '../..')

from utils import MathUtils
from acados_template import AcadosModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from casadi import *
import numpy as np
import scipy
from scipy import linalg
import time


class blasterModel: 

    def __init__(self, mass, J, l): 

        self._M = mass
        self._J = J
        self._arm_length = l
        self._c = 0.002

    def generateModel(self):

        """

            p -> position
            eta -> quaternions 
            omega -> angular velocity
            v -> linear velocity
            alpha -> motor 1 angle
            beta -> motor 2 angle 
            T_blast -> thruster due to blasting.

        """ 

        model_name = 'blasterModel'

        # Related to drone pose.

        self._p = SX.sym('p', 3)
        self._eta = SX.sym('eta', 4)
        self._omega = SX.sym('omega', 3)
        self._v = SX.sym('v', 3)

        # Related to motor angles. 

        self._alpha = SX.sym('alpha', 1)
        self._beta = SX.sym('beta', 1)
        self._Jac_p = SX.sym('J_p', 3, 3)
        self._Jac_eta = SX.sym('J_eta', 3, 4)
        self._Jac_angles = SX.sym('J_angle', 3, 2)

        # Related to external forces acting on drone.

        self._T = SX.sym('T', 4)
        self._T_blast = SX.sym('T_blast', 1)
        self._gravity = SX([0, 0, -9.81])

        """ 
            TODO: Change moment arm length. It should be vectors instead.    
        """

        self._MomentArm = 0 # Change this.
        self._Moments = vertcat(

            (- self._T[0] - self._T[3] + self._T[1] + self._T[2]) * self._arm_length, 
            (self._T[1] + self._T[3] - self._T[0] - self._T[2]) * self._arm_length, 
            (- self._T[0] - self._T[1] + self._T[2] + self._T[3]) * self._c

        )

        self._p_dot = self._v
        self._eta_dot = MathUtils.quatMultiplication(self._eta, vertcat(0, self._omega/2))
        self._v_dot = 1/self._M * MathUtils.quat2Rot(self._eta)*(self._T[0] + self._T[1] + self._T[2] + self._T[3] + self._T_blast) + self._gravity
        self._omega_dot = inv(self._J) @ (self._Moments - cross(self._omega, self._J @ self._omega))
        self._poc = self._Jac_p @ self._v + self._Jac_eta @ self._eta_dot + self._Jac_angles @ vertcat(self._alpha, self._beta)

    def generateController(self): 

        # Generate controller here. 

        pass 

if __name__ == "__main__": 

    b = blasterModel(7.5, np.eye(3) , 0.5)
    b.generateModel()