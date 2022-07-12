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

        """

            How to use this solver: 
            
                1. Based on UAV position, set initial conditions.
                    a. We need 4 main states. (x, y, z position of the drone and initial vel_x,y,z of the fluids out of the UAV.
                2. To solve for Jacobian, need to perform finite differences on: 
                    a. Orientation of vehicle, 
                    b. Orientation of nozzle, 
                    c. Position of vehicle. 
                3. Then, the Jacobians can be appended to the controller model to better predict 
                    a. POC 
                    b. Reaction forces acting on the UAV. 
        
        """ 

        self._streamVelocity = streamVelocity
        self._M_c = M_c
        self._Ts = Ts
        self._initConditions = np.zeros(6)

    def _getMag(self, vel, ang):
        angle = np.deg2rad(ang)
        anglex = vel*np.sin(angle)
        angley = 0
        anglez = -vel*np.cos(angle)
        return anglex, angley, anglez

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
        sim.solver_options.num_stages = 4
        sim.solver_options.num_steps = 10

        # Create integrator. 

        self.integrator = AcadosSimSolver(sim)

    def _solveForwardIntegration(self, initConditions, T): 

        # Set initial conditions
        self.integrator.set('T', T)
        self.integrator.set('x', initConditions)
        # Integrate 
        self.integrator.solve()
        sol = self.integrator.get('x')
        # Return states
        return sol


    def _solveRootFindingProblem(self, initialGuess, function): 

        """Initial guess is the amount of time to hit the ground.
        Function is the term that needs to be solved for zero."""

        # Use root-finding algorithm 
        #
        # Get time required for fluid to hit the ground

        # Takes about 0.005 - 0.09 seconds depending on how small the time step is with the old step. 
        # New method takes about 0.0007 seconds.

        T_N = initialGuess
        error = 100 # For initialisation sake
        steps_taken = 0

        while np.abs(error) > 1e-3: 

            T_Nplus1 = self._rootFindingStep(T_N, function)
            if T_Nplus1 < 0: 
                T_Nplus1 = 0
            error = function(T_Nplus1)
            T_N = T_Nplus1
            steps_taken += 1

        return T_Nplus1 

    def _rootFindingStep(self, T_N, function):

        # Just to solve 1 step.

        f = function(T_N)
        delta_T_N = 1e-1
        T_N_plus = T_N + delta_T_N
        f_prime = (function(T_N_plus) - f)/delta_T_N

        T_Nplus1 = T_N - (f / f_prime) 

        return T_Nplus1

    def solveJacobians(self, R_Nozzle): 

        """R_Nozzle: Numpy 3x3 array. Need to get the R_Nozzle to get initial velocity in x-y-z direction"""

        # Use finite-differences to get dot p_{POC}. 


        pass

    def setInitConditions(self, initConditions):

        self._initConditions = initConditions

    def getJacobians(self): 

        # This is the main Jacobians getter function. 
        
        pass

    def getInitConditions(self): 

        # Based on the pose of the drone and orientation of the nozzle, we can get the initial states of the system.

        initConditions = None

        return initConditions

    def _simulateBlastPlot(self, T): 

        N = int(T / self._Ts)

        data = np.zeros((N+1, 6))

        # run getInitConditions

        self.integrator.set('T', self._Ts)
        x = self._initConditions

        data[0, :] = x

        for i in range(N): 

            x = self._solveForwardIntegration(x, self._Ts)
            data[i+1, :] = x

        print(T)
        
        self.integrator.set('x', self._initConditions)
        self.integrator.set('T', 0.0416623212)
        # Integrate 
        self.integrator.solve()
        sol = self.integrator.get('x')
        print(sol)


        ax = plt.axes(projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.plot3D(data[:, 0], data[:, 1], data[:, 2])

        plt.show()

    def _function(self, T_N):

        """This function assumes that we are shooting on the ground. 
        If we are shooting onto the side of a wall, try: 
        return || self._simulateBlast(T_N)[0] + self._simulateBlast(T_N)[1] ||"""

        return self._solveForwardIntegration(self._initConditions, T_N)[2]

    def _solveFiniteDifferences(): 

        pass


if __name__ == "__main__":

    solver = Jacobian_POC_Solver(20, 1.0, 0.00015)
    solver._createIntegrator()
    mag = solver._getMag(152, 20)
    initConditions = np.array([0.5, 2.0, 4.0, mag[0], mag[1], mag[2]])
    solver.setInitConditions(initConditions)
    t0 = time.time()
    sol = solver._solveRootFindingProblem(0.001, solver._function)
    print("Time Elapsed: ",time.time() - t0)
    solver._simulateBlastPlot(sol)