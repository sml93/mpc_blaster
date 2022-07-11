import sys
sys.path.insert(0, '../utils')

from acados_template import AcadosModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from casadi import *
import numpy as np
import scipy
from scipy import linalg
import time


class blasterModel: 

    def __init__(self): 

        pass 

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

        p_dot = v
        # eta = 