from blastermodel import blasterModel 
from Jacobian_POC_Solver import Jacobian_POC_Solver
import numpy as np
import time
from matplotlib import pyplot as plt
from casadi import *

if __name__ == "__main__":

    # GENERATE REQUIRED CONTROLLER AND INTEGRATOR

    mass = 9.0
    J = np.eye(3)
    J[0, 0] = 0.50781
    J[1, 1] = 0.47314
    J[2, 2] = 0.72975
    l_x = 0.3434 
    l_y = 0.3475
    N = 60
    Tf = 2.0
    yaw_coefficient = 0.03
    blastThruster = 2.2*9.81
    Q = np.zeros((17, 17))
    np.fill_diagonal(Q, [1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 0.5e1, 0.5e1, 0.5e1, 1e1, 1e1, 1e1, 1e-2, 1e-2, 1e3, 1e3, 1e3]) # position, euler, velocity, angular velocity, swivel angles, POC.
    Q_t = 10*Q
    R = np.zeros((6, 6))
    np.fill_diagonal(R, [5e-2, 5e-2, 5e-2, 5e-2, 1e-5, 1e-5])
    statesBound = np.array([[-1.5, -1.5, 0, -0.174532925, -0.174532925, -0.349066, -1.0, -1.0, -1.0, -0.0872665, -0.0872665, -0.0872665, -0.174532925, -0.523599, -1.5, -1.5, -2.5],
                            [1.5, 1.5, 5.0, 0.174532925, 0.174532925, 0.349066, 1.0, 1.0, 1.0, 0.0872665, 0.0872665, 0.0872665, 1.22173, 0.523599, 1.5, 1.5, 2.5]])
    controlBound = np.array([[0, 0, 0, 0, -0.0872665, -0.0872665], [65, 65, 65, 65, 0.0872665, 0.0872665]])
    b = blasterModel(mass, J, l_x, l_y, N, Tf, yaw_coefficient, Q, R, Q_t, blastThruster, statesBound, controlBound)
    b.generateModel()
    integrator, ocp_solver = b.generateController()

    # GENERATE SIMULATION PARAMETERS

    solver = Jacobian_POC_Solver(150, 1, 0.000015)
    solver.initialise()
    J_mot, J_eul, J_pos = solver.getJacobians()
    
    nx = 17 
    nu = 6
    Nsim = 500
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim, nu))

    x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    yref = np.array([0.5, 0.0, 3.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0.0, 0, 0, 0, 0, 0.0, 0, 0])
    t = np.linspace(0, Tf/N*Nsim, Nsim+1)
    simX[0, :] = x0

    xcurrent = x0 
    
    # u = np.array([17, 17, 17, 17, 0.0, -0.05])

    for i in range(Nsim): 

        t0 = time.time()

        ocp_solver.set(0, "lbx", xcurrent)
        ocp_solver.set(0, "ubx", xcurrent)

        ocp_solver.cost_set(0, 'yref', yref)
        
        for k in range(N):
            
            params = np.vstack((np.reshape(J_mot, (J_mot.size, 1), order='F'), np.reshape(J_eul, (J_eul.size, 1), order='F'), np.reshape(J_pos, (J_pos.size, 1), order='F'), blastThruster))

            ocp_solver.set(k, 'p', params)
            

            if k+1 == N: 

                ocp_solver.cost_set(k+1, 'yref', yref[0:nx])

            else: 

                ocp_solver.cost_set(k+1, 'yref', yref)

        status = ocp_solver.solve()

        params = np.vstack((np.reshape(J_mot, (J_mot.size, 1), order='F'), np.reshape(J_eul, (J_eul.size, 1), order='F'), np.reshape(J_pos, (J_pos.size, 1), order='F'), 2.2*9.81))

        integrator.set('p', params)
        print(xcurrent)
        print(ocp_solver.get_cost())
        print(ocp_solver.get(0, "u"))

        simU[i,:] = ocp_solver.get(0, "u")

        # u = np.array([])

        # simulate system
        integrator.set("x", xcurrent)
        integrator.set("u", simU[i,:])
        # integrator.set("u", u)
        # integrator.set("u", np.array([22.0725, 22.0725, 22.0725, 22.0725, 0, 0]))

        status = integrator.solve()
        if status != 0:
            raise Exception('acados integrator returned status {}. Exiting.'.format(status))

        # update state
        xcurrent = integrator.get("x")
        simX[i+1,:] = xcurrent

        print(f"Time per step: {time.time() - t0}")

    plt.plot(t, simX[:, 0], label='x')
    plt.plot(t, simX[:, 1], label='y')
    plt.plot(t, simX[:, 2], label='z')
    plt.legend()
    plt.show()

    plt.plot(t, simX[:, 14], label='POC_{x}')
    plt.plot(t, simX[:, 15], label='POC_{y}')
    plt.plot(t, simX[:, 16], label='POC_{z}')
    plt.legend()
    plt.show()

    plt.plot(t, np.rad2deg(simX[:, 3]), label='phi')
    plt.plot(t, np.rad2deg(simX[:, 4]), label='tetha')
    plt.plot(t, np.rad2deg(simX[:, 5]), label='psi')
    plt.legend()
    plt.show()

    plt.plot(t, np.rad2deg(simX[:, 12]), label='alpha1')
    plt.plot(t, np.rad2deg(simX[:, 13]), label='alpha2')
    plt.legend()
    plt.show()