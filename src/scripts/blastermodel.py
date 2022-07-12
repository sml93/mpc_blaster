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

    def __init__(self, mass, J): 

        self._M = mass
        self._J = J


    def generateModel(self):

        """

            p -> position
            eta -> quaternions 
            omega -> angular velocity
            v -> linear velocity
            alpha -> motor 1 angle
            beta -> motor 2 angle 

        """ 

        model_name = 'blasterModel'
        p = SX.sym('p', 3)
        eta = SX.sym('eta', 4)
        omega = SX.sym('omega', 3)
        v = SX.sym('v', 3)
        alpha = SX.sym('alpha', 1)
        beta = SX.sym('beta', 1)
        T = SX.sym('T', 4)
        gravity = SX([0, 0, -9.81])

        p_dot = v
        eta_dot = MathUtils.quatMultiplication(eta, vertcat(0, omega/2))
        v_dot = 1/self._M * MathUtils.quatMultiplication(MathUtils.quatMultiplication(eta, vertcat(0, 0, 0, T[0] + T[1] + T[2] + T[3])), MathUtils.unitQuatInversion(eta))[1:4] + gravity
        print(MathUtils.quatMultiplication(MathUtils.quatMultiplication(eta, vertcat(0, 0, 0, T[0] + T[1] + T[2] + T[3])), MathUtils.unitQuatInversion(eta)))
        omega_dot = 0

if __name__ == "__main__": 

    b = blasterModel(7.5, 0)
    b.generateModel()