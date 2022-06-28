import sys
sys.path.insert(0, '../classes')
sys.path.insert(0, '../utils')

import shutil
from acados_template import AcadosModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from casadi import *
import numpy as np
import scipy
from scipy import linalg
import time

class Jacobian_POC_Solver: 


    def __init__(self, streamVelocity): 

        self._streamVelocity = streamVelocity

    def _createIntegrator(self): 

        # Initialise states
        # 
        # Create model 
        # 
        # Create integrator. 

        pass

    def _solveForwardIntegration(self): 

        # Set initial conditions
        #
        # Integrate 
        #
        # Return states

        pass

    def _solveRootFindingProblem(self): 

        # Use root-finding algorithm 
        #
        # Get time required for fluid to hit the ground

        pass

    def solveJacobians(self, R_Nozzle): 

        """R_Nozzle: Numpy 3x3 array. Need to get the R_Nozzle to get initial velocity in x-y-z direction"""

        # Use finite-differences to get dot p_{POC}. 


        pass

    def getJacobians(self): 

        # This is the main Jacobians getter function. 
        
        pass


if __name__ == "__main__":

    Jacobian_POC_Solver(20)