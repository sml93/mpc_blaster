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

    def __init__(self, mass, J, l_x, l_y, N): 

        self._M = mass
        self._J = J
        self._arm_length_x = l_x
        self._arm_length_y = l_y
        self._c = 0.002
        self._N = N

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

        self._model_name = 'blasterModel'

        # Related to drone pose.

        self._p = SX.sym('p', 3)
        self._eta = SX.sym('eta', 4)
        self._omega = SX.sym('omega', 3)
        self._v = SX.sym('v', 3)
        self._poc = SX.sym('poc', 3)

        # Related to motor angles. 

        self._alpha_dot = SX.sym('alpha', 1)
        self._beta_dot = SX.sym('beta', 1)
        self._Jac_p = SX.sym('J_p', 3, 3)
        self._Jac_eta = SX.sym('J_eta', 3, 4)
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

        self._p_dot = self._v
        self._eta_dot = MathUtils.quatMultiplication(self._eta, vertcat(0, self._omega/2))
        self._v_dot = 1/self._M * MathUtils.quat2Rot(self._eta) @ vertcat(0, 0, (self._T[0] + self._T[1] + self._T[2] + self._T[3] + self._T_blast)) + self._gravity
        self._omega_dot = inv(self._J) @ (self._Moments - cross(self._omega, self._J @ self._omega))
        self._poc_dot = self._Jac_p @ self._v + self._Jac_eta @ self._eta_dot + self._Jac_angles @ vertcat(self._alpha_dot, self._beta_dot)

        self._model = AcadosModel()
        self._model.name = self._model_name
        self._model.x = vertcat(

            self._p,
            self._eta,
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
            self._eta_dot,
            self._v_dot,
            self._omega_dot,
            self._poc_dot
        
        )
        self._model.z = [] 
        self._model.p = vertcat(

            self._Jac_p,
            self._Jac_eta,
            self._Jac_angles

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

            TODO: 
                1. Fix weights. 
                2. Find out where yref is for.
                3. Set constraints

        """

        ocp.cost.W = scipy.linalg.block_diag(self._Q_weight, self._Q_weight_, self._R_weight)
        ocp.cost.W_e = scipy.linalg.block_diag(self._Q_weight_t, self._Q_weight_t_)

        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)

        Vu = np.zeros((ny, nu))
        Vu[nx:, :] = np.eye(nu)
        ocp.cost.Vu = Vu

        ocp.cost.Vx_e = np.eye(nx)

        ocp.cost.yref = np.zeros((ny, ))
        ocp.cost.yref_e = np.zeros(ny_e, )

        ocp.constraints.idxbu = np.array([i for i in range(nu)])
        ocp.constraints.lbu = np.array([-self._qdot_max for i in range(nu)])
        ocp.constraints.ubu = np.array([self._qdot_max for i in range(nu)])
        ocp.constraints.x0 = np.zeros(nx)

        ocp.constraints.idxbx = np.array([i+self._n_states for i in range(nx - self._n_states)])
        ocp.constraints.lbx = np.array([0 for i in range(nx - self._n_states)])
        ocp.constraints.ubx = np.array([self._q_max for i in range(nx - self._n_states)])

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

    b = blasterModel(7.5, np.eye(3), 0.25, 0.25, 100)
    b.generateModel()