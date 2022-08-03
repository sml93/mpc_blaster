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
from htm import compute_T_b_s2, compute_T_w_b

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

        self._eps = 1e-6
        self._streamVelocity = np.array([0, 0, streamVelocity])
        self._M_c = M_c
        self._Ts = Ts
        self._initConditions = np.zeros(6)

        self._euler_angles = np.zeros(3)
        self._motor_angles = np.zeros(2)
        self._positions = np.zeros(3)

        self._POC = np.zeros(3)

        self._J_pos = np.zeros((3, 3))
        self._J_eul = np.zeros((3, 3))
        self._J_mot = np.zeros((3, 2))

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
        sim.solver_options.sens_forw = False
        # sim.solver_options.sens_hess = True
        # sim.solver_options.sens_adj = True

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

    def _solveRootFindingProblem(self, initialGuess, function, initConditions):
        """Initial guess is the amount of time to hit the ground.
        Function is the term that needs to be solved for zero."""

        # Use root-finding algorithm
        #
        # Get time required for fluid to hit the ground

        # Takes about 0.005 - 0.09 seconds depending on how small the time step is with the old step.
        # New method takes about 0.0007 seconds.

        T_N = initialGuess
        error = 100  # For initialisation sake
        steps_taken = 0

        while np.abs(error) > 1e-3:

            T_Nplus1 = self._rootFindingStep(T_N, function, initConditions)
            if T_Nplus1 < 0:
                T_Nplus1 = -T_Nplus1
            error = function(T_Nplus1, initConditions)
            T_N = T_Nplus1
            steps_taken += 1

        return T_Nplus1

    def _rootFindingStep(self, T_N, function, initConditions):

        # Just to solve 1 step.

        f = function(T_N, initConditions)
        delta_T_N = 1e-1
        T_N_plus = T_N + delta_T_N
        f_prime = (function(T_N_plus, initConditions) - f) / delta_T_N

        T_Nplus1 = T_N - (f / f_prime)

        return T_Nplus1

    def setInitConditions(self, euler_angles, motor_angles, position):

        self._euler_angles = euler_angles
        self._motor_angles = motor_angles
        self._positions = position
        T = compute_T_w_b(euler_angles[0], euler_angles[1], euler_angles[
                          2], position) @ compute_T_b_s2(motor_angles[0], motor_angles[1])
        p = T[0:3, 3]
        R = T[0:3, 0:3]
        initConditions = np.hstack((p, R@self._streamVelocity))

        self._initConditions = initConditions

    def setInitConditions_Plus(self, euler_angles, motor_angles, position):

        T = compute_T_w_b(euler_angles[0], euler_angles[1], euler_angles[
                          2], position) @ compute_T_b_s2(motor_angles[0], motor_angles[1])
        p = T[0:3, 3]
        R = T[0:3, 0:3]
        initConditions = np.hstack((p, R@self._streamVelocity))

        return initConditions

    def getJacobians(self):

        # This is the main Jacobians getter function.

        pass

    def getInitConditions(self):

        # Based on the pose of the drone and orientation of the nozzle, we can
        # get the initial states of the system.

        initConditions = None

        return initConditions

    def _simulateBlastPlot(self, T):

        N = int(T / self._Ts)

        data = np.zeros((N + 1, 6))

        # run getInitConditions

        self.integrator.set('T', self._Ts)
        x = self._initConditions

        data[0, :] = x

        for i in range(N):

            x = self._solveForwardIntegration(x, self._Ts)
            data[i + 1, :] = x

        print(T)

        self.integrator.set('x', self._initConditions)
        self.integrator.set('T', 0.2421937)
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

    def _function(self, T_N, initConditions):
        """This function assumes that we are shooting on the ground. 
        If we are shooting onto the side of a wall, try: 
        return || self._simulateBlast(T_N)[0] + self._simulateBlast(T_N)[1] ||"""

        return self._solveForwardIntegration(initConditions, T_N)[2]

    def solveJacobians(self, euler_angles, motor_angles, position):
        """R_Nozzle: Numpy 3x3 array. Need to get the R_Nozzle to get initial velocity in x-y-z direction"""

        # Use finite-differences to get dot p_{POC}.

        t0 = time.time()

        self.setInitConditions(euler_angles, motor_angles, position)
        self.integrator.set('x', self._initConditions)
        self._Ts = self._solveRootFindingProblem(0.1, self._function, self._initConditions)
        self.integrator.set('T', self._Ts)
        self.integrator.solve()
        self._POC = self.integrator.get('x')[0:3]

        print("Time Elapsed 1: ", time.time() - t0)

        t0 = time.time()

        for i in range(3):

            eps = np.zeros(3)
            eps[i] = self._eps
            euler_angles = euler_angles + eps
            initConditions = self.setInitConditions_Plus(
                euler_angles, motor_angles, position)
            self.integrator.set('x', initConditions)
            self._Ts = self._solveRootFindingProblem(0.1, self._function, initConditions)
            self.integrator.set('T', self._Ts)
            self.integrator.solve()
            POC_plus = self.integrator.get('x')[0:3]
            J_i = self._solveFiniteDifferences(POC_plus, self._POC)
            self._J_eul[:, i] = J_i
            euler_angles[0:3] = self._euler_angles[0:3]

        print("Time Elapsed 2: ", time.time() - t0)

        for i in range(2):

            eps = np.zeros(2)
            eps[i] = self._eps
            motor_angles = motor_angles + eps
            initConditions = self.setInitConditions_Plus(
                euler_angles, motor_angles, position)
            self.integrator.set('x', initConditions)
            self._Ts = self._solveRootFindingProblem(0.1, self._function, initConditions)
            self.integrator.set('T', self._Ts)
            self.integrator.solve()
            POC_plus = self.integrator.get('x')[0:3]
            J_i = self._solveFiniteDifferences(POC_plus, self._POC)
            self._J_mot[:, i] = J_i
            motor_angles = self._motor_angles

        for i in range(3):

            eps = np.zeros(3)
            eps[i] = self._eps
            position += eps
            initConditions = self.setInitConditions_Plus(
                euler_angles, motor_angles, position)
            self.integrator.set('x', initConditions)
            self._Ts = self._solveRootFindingProblem(0.1, self._function, initConditions)
            self.integrator.set('T', self._Ts)
            self.integrator.solve()
            POC_plus = self.integrator.get('x')[0:3]
            J_i = self._solveFiniteDifferences(POC_plus, self._POC)
            self._J_pos[:, i] = J_i
            position = self._positions

    def _solveFiniteDifferences(self, POC_plus, POC):

        return (POC_plus - POC) / self._eps


if __name__ == "__main__":

    solver = Jacobian_POC_Solver(20, 1.0, 0.00015)
    solver._createIntegrator()
    solver.setInitConditions([0, 0, 0], [0, 0], [0, 0, 4])
    t0 = time.time()
    solver.solveJacobians([0, 0, 0], [0, 0], [0, 0, 4])
    print("Time Elapsed: ", time.time() - t0)
    sol = solver._solveRootFindingProblem(0.001, solver._function, solver._initConditions)
    
    solver._simulateBlastPlot(sol)
