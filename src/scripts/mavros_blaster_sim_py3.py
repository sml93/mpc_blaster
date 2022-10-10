import time
from mavros_blaster_sim import thrusterCumul
import rospy
import numpy as np

from matplotlib import pyplot as plt
from blastermodel import blasterModel
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import AttitudeTarget
from Jacobian_POC_Solver import Jacobian_POC_Solver
from geometry_msgs import *
# from tf.transformations import quaternion_from_euler
from transforms3d.euler import euler2quat as quaternion_from_euler
from transforms3d.euler import quat2euler


class Quadcopter_Controller:

  def __init__(self, quad_params):

    self.initialise_parameters(quad_params)
    self.initialise_MPC()
    self.initialise_node()

  def initialise_node(self):

    rospy.init_node('quadcopter_controller')

    self.initialise_publishers()
    self.initialise_subscribers()

    self.main_loop()

  def initialise_parameters(self, quad_params): 

    self.nx = 17 
    self.nu = 6 
    self.attitude_target_msg = AttitudeTarget()
    self.quadcopter_states = np.zeros(17)
    self.mass = quad_params['mass']
    self.J = quad_params['J']

  def initialise_publishers(self): 

    self.feedforward_attitude_publisher = rospy.Publisher('desired_atti', AttitudeTarget, queue_size=1) 
    self.POC_publisher = rospy.Publisher('/point_of_contact', Point, queue_size=1)

  def initialise_subscribers(self):

    self.odom_subscriber = rospy.Subscriber('/mavros/local_position/odom', PoseStamped, self.local_position_callback)
    # self.swivel_angles_subscriber = rospy.Subscriber('/swivel_angles', Float64MultiArray, self.swivel_angles)

  def initialise_MPC(self): 

    # Drone parameters

    self.N = 60
    Tf = 2.0
    Q = np.zeros((17, 17))
    np.fill_diagonal(Q, [1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 0.5e1, 0.5e1, 0.5e1, 1e1, 1e1, 1e1, 1e-2, 1e-2, 1e3, 1e3, 1e3]) # position, euler, velocity, angular velocity, swivel angles, POC.
    Q_t = 10*Q
    R = np.zeros((6, 6))
    np.fill_diagonal(R, [5e-2, 5e-2, 5e-2, 5e-2, 1e-5, 1e-5])
    statesBound = np.array([[-1.5, -1.5, 0, -0.174532925, -0.174532925, -0.349066, -1.0, -1.0, -1.0, -0.0872665, -0.0872665, -0.0872665, -0.174532925, -0.523599, -1.5, -1.5, -2.5],
                            [1.5, 1.5, 5.0, 0.174532925, 0.174532925, 0.349066, 1.0, 1.0, 1.0, 0.0872665, 0.0872665, 0.0872665, 1.22173, 0.523599, 1.5, 1.5, 2.5]])
    controlBound = np.array([[0, 0, 0, 0, -0.0872665, -0.0872665], [65, 65, 65, 65, 0.0872665, 0.0872665]])
    b = blasterModel(self.mass, self.J, self.l_x, self.l_y, self.N, Tf, self.yaw_coefficient, Q, R, Q_t, self.blastThruster, statesBound, controlBound)
    b.generateModel()
    self.integrator, self.ocp_solver = b.generateController()

  def local_position_callback(self, msg): 

    self.quadcopter_states[0] = msg.pose.position.x
    self.quadcopter_states[1] = msg.pose.position.y
    self.quadcopter_states[2] = msg.pose.position.z

    euler_angles = quat2euler([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z], 'szyx')

    self.quadcopter_states[3] = euler_angles[0]
    self.quadcopter_states[4] = euler_angles[1]
    self.quadcopter_states[5] = euler_angles[2]

    self.quadcopter_states[6] = msg.twist.linear.x
    self.quadcopter_states[7] = msg.twist.linear.y
    self.quadcopter_states[8] = msg.twist.linear.z

    self.quadcopter_states[9] = msg.twist.angular.x
    self.quadcopter_states[10] = msg.twist.angular.y
    self.quadcopter_states[11] = msg.twist.angular.z

  def update_solver(self):

    self.ocp_solver.set(0, "lbx", self.quadcopter_states)
    self.ocp_solver.set(0, "ubx", self.quadcopter_states)
  
    self.ocp_solver.cost_set(0, 'yref', self.yref)

    for k in range(self.N): 

      if k+1 == self.N: 

        self.ocp_solver.cost_set(k+1, 'yref', self.yref[0:self.nx])

      else: 

        self.ocp_solver.cost(k+1, 'yref', self.yref)

    print(f"ocp_solver.get_cost(): {self.ocp_solver.get_cost()}")

    self.attitude_target_msg.type_mask = 7
    target_quaternion = quaternion_from_euler(self.ocp_solver.get(0, "x")[3], self.ocp_solver.get(0, "x")[4], self.ocp_solver.get(0, "x")[5])
    self.attitude_target_msg.orientation.w = target_quaternion[0]
    self.attitude_target_msg.orientation.x = target_quaternion[1]
    self.attitude_target_msg.orientation.y = target_quaternion[2]
    self.attitude_target_msg.orientation.z = target_quaternion[3]

    self.attitude_target_msg.thrust = thrusterCumul(self.ocp_solver.get(0, "u")[0], self.ocp_solver.get(0, "u")[1], self.ocp_solver.get(0, "u")[2], self.ocp_solver.get(0, "u")[3])
    
    self.feedforward_attitude_publisher.publish(self.attitude_target_msg)

  def main_loop(self): 

    while not rospy.is_shutdown(): 

      self.update_solver()

    


    

















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
