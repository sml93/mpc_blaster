from blastermodel import blasterModel 
from Jacobian_POC_Solver import Jacobian_POC_Solver
import numpy as np
import time
from matplotlib import pyplot as plt

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
    Tf = 1.0
    yaw_coefficient = 0.03
    blastThruster = 2.2
    Q = np.zeros((17, 17))
    np.fill_diagonal(Q, [1e5, 1e5, 1e5, 1e2, 1e2, 1e2, 0.5e1, 0.5e1, 0.5e1, 1e1, 1e1, 1e1, 1e-2, 1e-2, 0.01e2, 0.01e2, 0.01e2]) # position, euler, velocity, angular velocity, swivel angles, POC.
    Q_t = 1*Q
    R = np.zeros((6, 6))
    np.fill_diagonal(R, [5e-3, 5e-3, 5e-3, 5e-3, 1e1, 1e1])
    statesBound = np.array([[-1.5, -1.5, 0, -0.174532925, -0.174532925, -0.349066, -0.5, -0.5, -0.5, -0.0872665, -0.0872665, -0.0872665, -0.174532925, -0.523599, -1.5, -1.5, -2.5],
                            [1.5, 1.5, 5.0, 0.174532925, 0.174532925, 0.349066, 0.5, 0.5, 0.5, 0.0872665, 0.0872665, 0.0872665, 1.22173, 0.523599, 1.5, 1.5, 2.5]])
    controlBound = np.array([[0, 0, 0, 0, -0.0872665, -0.0872665], [65, 65, 65, 65, 0.0872665, 0.0872665]])
    b = blasterModel(mass, J, l_x, l_y, N, Tf, yaw_coefficient, Q, R, Q_t, blastThruster, statesBound, controlBound)
    b.generateModel()
    integrator, ocp_solver = b.generateController()

    # GENERATE SIMULATION PARAMETERS

    nx = 17 
    nu = 6
    Nsim = 500
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim, nu))

    x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    yref = np.array([0, 0.0, 5.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    t = np.linspace(0, Tf/N*Nsim, Nsim+1)
    simX[0, :] = x0

    xcurrent = x0 

    for i in range(Nsim): 

        t0 = time.time()

        ocp_solver.set(0, "lbx", xcurrent)
        ocp_solver.set(0, "ubx", xcurrent)

        for k in range(N): 
        
            status = ocp_solver.cost_set(k, 'yref', yref)

        status = ocp_solver.solve()
        print(xcurrent)
        print(ocp_solver.get_cost())
        print(ocp_solver.get(0, "u"))

        simU[i,:] = ocp_solver.get(0, "u")

        # u = np.array([])

        # simulate system
        integrator.set("x", xcurrent)
        integrator.set("u", simU[i,:])
        # integrator.set("u", np.array([22.0725, 22.0725, 22.0725, 22.0725, 0, 0]))

        status = integrator.solve()
        if status != 0:
            raise Exception('acados integrator returned status {}. Exiting.'.format(status))

        # update state
        xcurrent = integrator.get("x")
        simX[i+1,:] = xcurrent

        print(f"Time per step: {time.time() - t0}")

    plt.plot(t, simX[:, 0:6])
    # plt.plot(t[0:Nsim], simU[:, 0:4])
    plt.show()