#!/usr/bin/env python2
import time
import rospy
import numpy as np

from matplotlib import pyplot as plt
from blastermodel import blasterModel
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget
from Jacobian_POC_Solver import Jacobian_POC_Solver
# from tf.transformations import quaternion_from_euler
from transforms3d.euler import euler2quat as quaternion_from_euler

# Drone variables
mass = 9.0
J = np.eye(3)
J[0, 0] = 0.50781
J[1, 1] = 0.47314
J[2, 2] = 0.72975
l_x = 0.3434 
l_y = 0.3475
yaw_coefficient = 0.03
blastThruster = 2.2
thrusterCoefficient = 2.3


def thrusterCumul(t1,t2,t3,t4):
  avg = thrusterCoefficient*np.mean([t1,t2,t3,t4])/9.81
  T_sp = (0.0014*np.power(avg,3)) - (0.0263*np.power(avg,2)) + (0.2464*avg) -0.0286
  return T_sp

def talker():
  pub = rospy.Publisher('desired_atti', AttitudeTarget, queue_size=10)
  rospy.init_node('mavros_blaster', anonymous=True)
  r = rospy.Rate(10)
  msg = AttitudeTarget()

  # Drone parameters
  N = 30
  Tf = 1.0
  Q = np.zeros((17, 17))
  np.fill_diagonal(Q, [1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 0.5e1, 0.5e1, 0.5e1, 1e1, 1e1, 1e1, 1e-2, 1e-2, 0.01e2, 0.01e2, 0.01e2]) # position, euler, velocity, angular velocity, swivel angles, POC.
  Q_t = 10*Q
  R = np.zeros((6, 6))
  np.fill_diagonal(R, [5e-2, 5e-2, 5e-2, 5e-2, 1e1, 1e1])
  statesBound = np.array([[-1.5, -1.5, 0, -0.174532925, -0.174532925, -0.349066, -0.5, -0.5, -0.5, -0.0872665, -0.0872665, -0.0872665, -0.174532925, -0.523599, -1.5, -1.5, -2.5],
                          [1.5, 1.5, 5.0, 0.174532925, 0.174532925, 0.349066, 0.4, 0.5, 1.0, 0.0872665, 0.0872665, 0.0872665, 1.22173, 0.523599, 1.5, 1.5, 2.5]])
  controlBound = np.array([[0, 0, 0, 0, -0.0872665, -0.0872665], [65, 65, 65, 65, 0.0872665, 0.0872665]])
  b = blasterModel(mass, J, l_x, l_y, N, Tf, yaw_coefficient, Q, R, Q_t, blastThruster, statesBound, controlBound)
  b.generateModel()
  integrator, ocp_solver = b.generateController()

  # Simulation parameters
  nx = 17 
  nu = 6
  Nsim = 750
  simX = np.ndarray((Nsim+1, nx))
  simU = np.ndarray((Nsim, nu))

  x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
  yref = np.array([0.5, 1.0, 3.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
  t = np.linspace(0, Tf/N*Nsim, Nsim+1)
  simX[0, :] = x0

  xcurrent = x0 

  for i in range(Nsim): 
    if not rospy.is_shutdown():
      t0 = time.time()

      ocp_solver.set(0, "lbx", xcurrent)
      ocp_solver.set(0, "ubx", xcurrent)

      ocp_solver.cost_set(0, 'yref', yref)

      for k in range(N):
          
        if k+1 == N: 

          status = ocp_solver.cost_set(k+1, 'yref', yref[0:nx])

        else: 
          status = ocp_solver.cost_set(k+1, 'yref', yref)

      status = ocp_solver.solve()
      print(xcurrent)
      print(ocp_solver.get_cost())
      print(ocp_solver.get(0, "u")[0:4])      # collective thrust
      print(ocp_solver.get(0, "x")[3:6])      # euler angles

      msg.type_mask = 7
      msg.orientation.w = quaternion_from_euler(ocp_solver.get(0, "x")[3], ocp_solver.get(0, "x")[4], ocp_solver.get(0, "x")[5])[0]
      msg.orientation.x = quaternion_from_euler(ocp_solver.get(0, "x")[3], ocp_solver.get(0, "x")[4], ocp_solver.get(0, "x")[5])[1]
      msg.orientation.y = quaternion_from_euler(ocp_solver.get(0, "x")[3], ocp_solver.get(0, "x")[4], ocp_solver.get(0, "x")[5])[2]
      msg.orientation.z = quaternion_from_euler(ocp_solver.get(0, "x")[3], ocp_solver.get(0, "x")[4], ocp_solver.get(0, "x")[5])[3]
      t1 = ocp_solver.get(0, "u")[0]
      t2 = ocp_solver.get(0, "u")[1]
      t3 = ocp_solver.get(0, "u")[2]
      t4 = ocp_solver.get(0, "u")[3]
      msg.thrust = thrusterCumul(t1,t2,t3,t4)
      # print(thrusterCumul(t1,t2,t3,t4))
      pub.publish(msg)

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

    # # plt.plot(t, simX[:, 6:9])
    # plt.plot(t, simX[:, 0:3])
    # # plt.plot(t[0:Nsim], simU[:, 0:4])
    # plt.show()

  msg.orientation.w = quaternion_from_euler(0,0,0)[0]
  msg.orientation.x = quaternion_from_euler(0,0,0)[1]
  msg.orientation.y = quaternion_from_euler(0,0,0)[2]
  msg.orientation.z = quaternion_from_euler(0,0,0)[3]
  msg.thrust = 0.705
  pub.publish(msg)


if __name__ == '__main__':
  try: talker()
  except rospy.ROSInterruptException:
    pass


















  # # Drone parameters
  # N = 30
  # Tf = 1.0
  # Q = np.zeros((17, 17))
  # np.fill_diagonal(Q, [1e2, 1e2, 1e2, 1e2, 1e2, 1e2, 0.5e1, 0.5e1, 0.5e1, 1e1, 1e1, 1e1, 1e-2, 1e-2, 0.01e2, 0.01e2, 0.01e2]) # position, euler, velocity, angular velocity, swivel angles, POC.
  # Q_t = 10*Q
  # R = np.zeros((6, 6))
  # np.fill_diagonal(R, [5e-2, 5e-2, 5e-2, 5e-2, 1e1, 1e1])
  # statesBound = np.array([[-1.5, -1.5, 0, -0.174532925, -0.174532925, -0.349066, -0.5, -0.5, -0.5, -0.0872665, -0.0872665, -0.0872665, -0.174532925, -0.523599, -1.5, -1.5, -2.5],
  #                         [1.5, 1.5, 5.0, 0.174532925, 0.174532925, 0.349066, 0.4, 0.5, 1.0, 0.0872665, 0.0872665, 0.0872665, 1.22173, 0.523599, 1.5, 1.5, 2.5]])
  # controlBound = np.array([[0, 0, 0, 0, -0.0872665, -0.0872665], [65, 65, 65, 65, 0.0872665, 0.0872665]])
  # b = blasterModel(mass, J, l_x, l_y, N, Tf, yaw_coefficient, Q, R, Q_t, blastThruster, statesBound, controlBound)
  # b.generateModel()
  # integrator, ocp_solver = b.generateController()

  # # Simulation parameters
  # nx = 17 
  # nu = 6
  # Nsim = 1000
  # simX = np.ndarray((Nsim+1, nx))
  # simU = np.ndarray((Nsim, nu))

  # x0 = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
  # yref = np.array([0.5, 1.0, 3.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
  # t = np.linspace(0, Tf/N*Nsim, Nsim+1)
  # simX[0, :] = x0

  # xcurrent = x0 

  # for i in range(Nsim): 

  #   t0 = time.time()

  #   ocp_solver.set(0, "lbx", xcurrent)
  #   ocp_solver.set(0, "ubx", xcurrent)

  #   ocp_solver.cost_set(0, 'yref', yref)

  #   for k in range(N):
        
  #     if k+1 == N: 

  #       status = ocp_solver.cost_set(k+1, 'yref', yref[0:nx])

  #     else: 
  #       status = ocp_solver.cost_set(k+1, 'yref', yref)

  #   status = ocp_solver.solve()
  #   print(xcurrent)
  #   print(ocp_solver.get_cost())
  #   print(ocp_solver.get(0, "u")[0:4])      # collective thrust
  #   print(ocp_solver.get(0, "x")[3:6])      # euler angles

  #   simU[i,:] = ocp_solver.get(0, "u")

  #   # u = np.array([])

  #   # simulate system
  #   integrator.set("x", xcurrent)
  #   integrator.set("u", simU[i,:])
  #   # integrator.set("u", np.array([22.0725, 22.0725, 22.0725, 22.0725, 0, 0]))

  #   status = integrator.solve()
  #   if status != 0:
  #       raise Exception('acados integrator returned status {}. Exiting.'.format(status))

  #   # update state
  #   xcurrent = integrator.get("x")
  #   simX[i+1,:] = xcurrent

  #   print(f"Time per step: {time.time() - t0}")

  # # plt.plot(t, simX[:, 6:9])
  # plt.plot(t, simX[:, 0:3])
  # # plt.plot(t[0:Nsim], simU[:, 0:4])
  # plt.show()
