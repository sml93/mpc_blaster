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

    def __init__(self, mass, J, l_x, l_y, N, c, thrustRange, Q, R, Q_t, blastThruster, statesBound, controlBound): 

        """
        
            Types:

                N: int 
                mass, l_x, l_y, c, blastThruster: double, 
                J: numpy 3x3 matrix.
                Q, Q_t: numpy 16x16 matrix.
                R: numpy 6x6 matrix.
                statesBound: 16x2 matrix. statesBound[0] -> lower bounds, statesBound[1] -> upper bounds.
                controlBound: 6x2 matrix. controlBound[0] -> lower bounds, controlBound[1] -> upper bounds.

        
        """

        self._M = mass
        self._J = J
        self._arm_length_x = l_x
        self._arm_length_y = l_y
        self._c = c
        self._N = N
        self._minThrust = thrustRange[0]
        self._maxThrust = thrustRange[1]
        self._Q_weight = Q  # Weight Matrix for states
        self._Q_weight_t = Q_t # Weight Matrix for terminal states
        self._R_weight = R  # Weight Matrix for control
        self._blastThruster = blastThruster
        self._statesBound = statesBound
        self._controlBound = controlBound

    def generateModel(self):

        """

            p -> position
            eta -> quaternions 
            omega -> angular velocity
            v -> linear velocity
            alpha -> motor 1 angle
            beta -> motor 2 angle 
            T_blast -> thruster due to blasting.
            Total number of states -> 16. p -> 3, q -> 4, v -> 3, omega -> 3, poc -> 3.
            Total number of controlled variables -> 6, 4 -> thrust motors, 2 swivel motors.
            Need to convert motor thrusts to rates + collective thrust.

        """ 

        self._model_name = 'blasterModel'

        # Related to drone pose.

        self._p = SX.sym('p', 3)
        self._phi = SX.sym('phi', 1)
        self._theta = SX.sym('theta', 1)
        self._psi = SX.sym('psi', 1)
        self._omega = SX.sym('omega', 3)
        self._v = SX.sym('v', 3)
        self._poc = SX.sym('poc', 3)

        # Related to motor angles. 

        self._alpha_dot = SX.sym('alpha', 1)
        self._beta_dot = SX.sym('beta', 1)
        self._Jac_p = SX.sym('J_p', 3, 3)
        self._Jac_euler = SX.sym('J_eta', 3, 3)
        self._Jac_angles = SX.sym('J_angle', 3, 2)

        # Related to external forces acting on drone.

        self._T = SX.sym('T', 4)
        self._T_blast = SX.sym('T_blast', 1)
        self._gravity = SX([0, 0, -9.81])

        self._Moments = vertcat(

            (- self._T[0] - self._T[3] + self._T[1] + self._T[2]) * self._arm_length_y, 
            (self._T[1] + self._T[3] - self._T[0] - self._T[2]) * self._arm_length_x, 
            (- self._T[0] - self._T[1] + self._T[2] + self._T[3]) * self._c

        )

        R_phi = SX.eye(3)
        R_theta = SX.eye(3)
        R_psi = SX.eye(3)

        R_phi[1, 1] = cos(self._phi)
        R_phi[1, 2] = -sin(self._phi)
        R_phi[2, 1] = sin(self._phi)
        R_phi[2, 2] = cos(self._phi)

        R_theta[0, 0] = cos(self._theta)
        R_theta[0, 2] = sin(self._theta)
        R_theta[2, 0] = -sin(self._theta)
        R_theta[2, 2] = cos(self._theta)

        R_psi[0, 0] = cos(self._psi)
        R_psi[0, 1] = -sin(self._psi)
        R_psi[1, 0] = sin(self._psi)
        R_psi[1, 1] = cos(self._psi)

        self._R = R_psi @ R_theta @ R_phi 

        self._p_dot = self._v
        self._phi_dot = 0
        self._theta_dot = 0
        self._psi_dot = 0 
        R_to_omega = SX.zeros(3, 3)
        R_to_omega[0, 0] = -sin(self._theta)
        R_to_omega[0, 2] = 1
        R_to_omega[1, 0] = sin(self._phi)*cos(self._theta)
        R_to_omega[1, 1] = cos(self._phi)
        R_to_omega[2, 0] = cos(self._phi)*cos(self._theta)
        R_to_omega[2, 1] = -sin(self._phi)

        self._euler_angles_dot = inv(R_to_omega)@self._omega
        self._v_dot = 1/self._M * self._R @ vertcat(0, 0, (self._T[0] + self._T[1] + self._T[2] + self._T[3] + self._T_blast)) + self._gravity
        self._omega_dot = inv(self._J) @ (self._Moments - cross(self._omega, self._J @ self._omega))
        self._poc_dot = self._Jac_p @ self._v + self._Jac_euler @ self._euler_angles_dot + self._Jac_angles @ vertcat(self._alpha_dot, self._beta_dot)

        self._model = AcadosModel()
        self._model.name = self._model_name
        self._model.x = vertcat(

            self._p,
            self._phi,
            self._theta,
            self._psi,
            self._v,
            self._omega,
            self._poc

        )
        self._model.u = vertcat(

            self._T,
            self._alpha_dot,
            self._beta_dot

        )
        self._model.f_expl_expr = vertcat(
            
            self._p_dot,
            self._euler_angles_dot,
            self._v_dot,
            self._omega_dot,
            self._poc_dot
        
        )
        self._model.z = [] 
        self._model.p = vertcat(

            self._Jac_p,
            self._Jac_eta,
            self._Jac_angles,
            self._T_blast

        )

        return 0

    def generateController(self): 

        # Generate controller here. 

        ocp = AcadosOcp()
        ocp.model = self._model

        nx = ocp.model.x.size()[0]
        nu = ocp.model.u.size()[0]
        ny = nx + nu 
        ny_e = nx 

        ocp.dims.N = self._N

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        """ 
        
            Define weightage here.

            Q -> 16 x 16, R -> 6 x 6. Q and R are weightage matrices.
            Vx -> 22 x 16. Vu -> 22 x 6. V is the selection matrix.
            yref is just default reference points.
        
            Indices for states are in this order: p, q, v, omega, poc.
            Indices for controls are in this order: T1, .., T4, alpha, beta.
         
        """

        ocp.cost.W = scipy.linalg.block_diag(self._Q_weight, self._R_weight)
        ocp.cost.W_e = scipy.linalg.block_diag(self._Q_weight_t)

        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)

        Vu = np.zeros((ny, nu))
        Vu[nx:, :] = np.eye(nu)
        ocp.cost.Vu = Vu

        ocp.cost.Vx_e = np.eye(nx)

        ocp.cost.yref = np.zeros((ny, ))
        ocp.cost.yref_e = np.zeros(ny_e, )

        # Setting the constraints here.

        ocp.constraints.idxbu = np.array([i for i in range(nu)]) # Setting indices for motors and swivel angles.

        ocp.constraints.lbu = self._controlBound[0] # Setting bounds for thrusts, then swivel angles.
        ocp.constraints.ubu = self._controlBound[1]

        ocp.constraints.x0 = np.zeros(nx)

        ocp.constraints.idxbx = np.array([i for i in range(nx)])
        ocp.constraints.lbx = self._statesBound[0]
        ocp.constraints.ubx = self._statesBound[1]

        ocp.solver_options.levenberg_marquardt = 0.0
        # ocp.solver_options.regularize_method = 'CONVEXIFY'
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  # FULL_CONDENSING_QPOASES
        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'  # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # SQP_RTI
        ocp.solver_options.nlp_solver_max_iter = 200
        ocp.parameter_values = np.zeros(self._n_states*self._n_tendons)
        # ocp.solver_options.qp_solver_cond_N = self._N

        # set prediction horizon
        ocp.solver_options.tf = self._Tf

        return 0

if __name__ == "__main__": 

    b = blasterModel(7.5, np.eye(3), 0.25, 0.25, 30, 0, [0, 5], 0, 0, 0, 0, 0, 0)
    b.generateModel()