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

from matplotlib import pyplot as plt

class Jacobian_POC_Solver: 


    def __init__(self, streamVelocity, M_c, Ts): 

        self._streamVelocity = streamVelocity
        self._M_c = M_c
        self._Ts = Ts

    def _createIntegrator(self): 

        # Initialise states

        model = AcadosModel()

        model_name = 'POC_Integrator'

        self._p = SX.sym('p', 3)
        self._v = SX.sym('v', 3)
        self._g = SX([0, 0, -9.81])

        self._p_dot = SX.sym('p_dot', 3)
        self._v_dot = SX.sym('v_dot', 3)

        states = vertcat(self._p, self._v)
        statesdot = vertcat(self._p_dot, self._v_dot)

        v_dot = - self._M_c @ self._v + self._g

        # Create model 

        model.name = model_name
        model.x = states 
        model.xdot = statesdot
        model.f_expl_expr = vertcat(self._v, v_dot)
        model.f_impl_expr = statesdot - vertcat(self._v, v_dot)
        model.u = SX([])
        model.z = []

        sim = AcadosSim()

        sim.model = model
        sim.solver_options.T = self._Ts
        sim.solver_options.integrator_type = 'ERK'
        sim.solver_options.num_stages = 2
        sim.solver_options.num_steps = 1

        # Create integrator. 

        self.integrator = AcadosSimSolver(sim)

    def _solveForwardIntegration(self, initConditions): 

        # Set initial conditions
        
        self.integrator.set('x', initConditions)
        # Integrate 
        self.integrator.solve()
        sol = self.integrator.get('x')
        # Return states
        return sol

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

    def getInitConditions(self): 

        # Based on the pose of the drone and orientation of the nozzle, we can get the initial states of the system.

        initConditions = None

        return initConditions

    def _simulateBlast(self, T): 

        N = T / self._Ts

    def _simulateBlastPlot(self, T, initConditions): 

        N = int(T / self._Ts)

        data = np.zeros((N+1, 6))

        # run getInitConditions

        x = initConditions

        data[0, :] = x

        for i in range(N): 

            x = self._solveForwardIntegration(x)
            data[i+1, :] = x

        ax = plt.axes(projection='3d')
        # ax.set_xlim3d([-0.3, 0.3])
        ax.set_xlabel('X')

        # ax.set_ylim3d([-0.3, 0.3])
        ax.set_ylabel('Y')

        # ax.set_zlim3d([0.0, 0.5])
        ax.set_zlabel('Z')
        ax.plot3D(data[:, 0], data[:, 1], data[:, 2])

        plt.show()


if __name__ == "__main__":

    solver = Jacobian_POC_Solver(20, 1.0, 0.01)
    solver._createIntegrator()
    initConditions = np.array([0.5, 2.0, 4.0, 140.9550, 0, -51.30])
    solver._simulateBlastPlot(0.1, initConditions)
