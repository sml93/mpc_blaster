#!/usr/bin/env python2
import time

from sympy import euler
import rospy
import numpy as np

from mavros import setpoint as SP
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
from blastermodel import blasterModel
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Quaternion, Point
from Jacobian_POC_Solver import Jacobian_POC_Solver
# from tf.transformations import quaternion_from_euler
from transforms3d.euler import euler2quat as quaternion_from_euler
from transforms3d.euler import quat2euler as quaternion_to_euler


class blasterController:
  def __init__(self, blaster_params):
    self.init_params(blaster_params)
    self.init_MPC()
    self.init_nodes()

  def init_nodes(self):
    rospy.init_node('blaster_controller')
    self.rate = rospy.Rate(20)
    self.init_pubsubs()
    self.run_node()
    self.rate.sleep()

  def thrusterCumul(self, t1, t2, t3, t4):
    avg = thrusterCoefficient*np.mean([t1,t2,t3,t4])/9.81
    T_sp = (0.0014*np.power(avg,3)) - (0.0263*np.power(avg,2)) + (0.2464*avg) -0.0286
    if (T_sp >= 1.0):
      T_sp = 0.8
    print('Thrust: ', round(T_sp,3))
    return T_sp

  def init_params(self, blaster_params):
    self.nx = 17
    self.nu = 6
    # self.N = 100
    self.blaster_states = np.zeros(17)
    self.yref = blaster_params['yref']
    self.mass = blaster_params['mass']
    self.J = blaster_params['J']
    self.l_x = blaster_params['l_x']
    self.l_y = blaster_params['l_y']
    self.N = blaster_params['N']
    self.yaw_coeff = blaster_params['yaw_coefficient']
    self.blastThruster = blaster_params['blastThruster']

  def init_pubsubs(self):
    self.ff_atti_pub = rospy.Publisher('/desired_atti', AttitudeTarget, queue_size=20)
    # self.POC_pub = rospy.Publisher(/poc, Point, queue_size=1)
    self.odom_sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, self.callback)
    # self.swivel_angles_subscriber = rospy.Subscriber('/swivel_angles', Float64MultiArray, self.swivel_angles)
    self.attitude_target_msg = AttitudeTarget(
      header = SP.Header(
        frame_id="base_footprint",
        stamp=rospy.Time.now()),
      )

  def init_MPC(self):
    # init drone params
    Tf = 2.0
    Q = np.zeros((17, 17))
    np.fill_diagonal(Q, [1e3, 1e3, 1e3, 1e3, 1e3, 1e3, 0.5e1, 0.5e1, 0.5e1, 1e1, 1e1, 1e1, 1e-10, 1e-10, 1e-10, 1e-10, 1e-10]) # position, euler, velocity, angular velocity, swivel angles, POC.
    Q_t = 10*Q
    R = np.zeros((6, 6))
    np.fill_diagonal(R, [5e-2, 5e-2, 5e-2, 5e-2, 1e-5, 1e-5])
    statesBound = np.array([[-5, -5, -1.0, -0.174532925, -0.174532925, -0.349066, -1.0, -1.0, -0.5, -0.0872665, -0.0872665, -0.0872665, -0.174532925, -0.523599, -5, -5, -1.0],
                            [5, 5, 10, 0.174532925, 0.174532925, 0.349066, 1.0, 1.0, 0.5, 0.0872665, 0.0872665, 0.0872665, 1.22173, 0.523599, 5, 5, 10.0]])
    controlBound = np.array([[0, 0, 0, 0, -0.0872665, -0.0872665], [65, 65, 65, 65, 0.0872665, 0.0872665]])
    b = blasterModel(self.mass, self.J, self.l_x, self.l_y, self.N, Tf, self.yaw_coeff, Q, R, Q_t, self.blastThruster, statesBound, controlBound)
    b.generateModel()
    self.integrator, self.ocp_solver = b.generateController()

    solver = Jacobian_POC_Solver(150, 1, 0.000015)
    solver.initialise()
    self.J_mot, self.J_eul, self.J_pos = solver.getJacobians()


  def callback(self, msg):
    self.blaster_states[0] = round(msg.pose.pose.position.x, 3)
    self.blaster_states[1] = round(msg.pose.pose.position.y, 3)
    self.blaster_states[2] = round(msg.pose.pose.position.z, 3)

    euler_angles = quaternion_to_euler([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])

    self.blaster_states[3] = round(euler_angles[0], 3)
    self.blaster_states[4] = round(euler_angles[1], 3)
    self.blaster_states[5] = round(euler_angles[2], 3)

    self.blaster_states[6] = round(msg.twist.twist.linear.x, 3)
    self.blaster_states[7] = round(msg.twist.twist.linear.y, 3)
    self.blaster_states[8] = round(msg.twist.twist.linear.z, 3)

    self.blaster_states[9] = round(msg.twist.twist.angular.x, 3)
    self.blaster_states[10] = round(msg.twist.twist.angular.y, 3)
    self.blaster_states[11] = round(msg.twist.twist.angular.z, 3)

  def update_solver(self):
    self.ocp_solver.set(0, "lbx", self.blaster_states)
    self.ocp_solver.set(0, "ubx", self.blaster_states)
    self.ocp_solver.cost_set(0, 'yref', self.yref)

    for k in range(self.N): 
      params = np.vstack((np.reshape(self.J_mot, (self.J_mot.size, 1), order='F'), np.reshape(self.J_eul, (self.J_eul.size, 1), order='F'), np.reshape(self.J_pos, (self.J_pos.size, 1), order='F'), self.blastThruster))
      self.ocp_solver.set(k, 'p', params)

      if k+1 == self.N: 
        self.ocp_solver.cost_set(k+1, 'yref', self.yref[0:self.nx])
      else: 
        self.ocp_solver.cost_set(k+1, 'yref', self.yref)

    self.ocp_solver.solve()
    params = np.vstack((np.reshape(self.J_mot, (self.J_mot.size, 1), order='F'), np.reshape(self.J_eul, (self.J_eul.size, 1), order='F'), np.reshape(self.J_pos, (self.J_pos.size, 1), order='F'), self.blastThruster))
    # self.integrator.set('p', params)
    print(f"ocp_solver.get_cost(): {self.ocp_solver.get_cost()}")
    
    self.attitude_target_msg.type_mask = 7
    target_quaternion = quaternion_from_euler(self.ocp_solver.get(0, "x")[3], self.ocp_solver.get(0, "x")[4], self.ocp_solver.get(0, "x")[5])
    self.attitude_target_msg.orientation.w = target_quaternion[0]
    self.attitude_target_msg.orientation.x = target_quaternion[1]
    self.attitude_target_msg.orientation.y = target_quaternion[2]
    self.attitude_target_msg.orientation.z = target_quaternion[3]

    print('u0: ', round(self.ocp_solver.get(0, "u")[0],3))
    print('u1: ', round(self.ocp_solver.get(0, "u")[1],3))
    print('u2: ', round(self.ocp_solver.get(0, "u")[2],3))
    print('u3: ', round(self.ocp_solver.get(0, "u")[3],3))

    self.attitude_target_msg.thrust = self.thrusterCumul(self.ocp_solver.get(0, "u")[0], self.ocp_solver.get(0, "u")[1], self.ocp_solver.get(0, "u")[2], self.ocp_solver.get(0, "u")[3])
    
    self.ff_atti_pub.publish(self.attitude_target_msg)

  def run_node(self):
    while not rospy.is_shutdown():
      self.update_solver()
      # self.rate.sleep()


if __name__ == '__main__':
  # Drone variables
  mass = 8.0
  J = np.eye(3)
  J[0, 0] = 0.50781
  J[1, 1] = 0.50781   #0.47314
  J[2, 2] = 0.72975
  l_x = 0.3434 
  l_y = 0.3475
  yaw_coefficient = 0.03
  # blastThruster = 2.2*9.81
  blastThruster = 0.0
  thrusterCoefficient = 3.7
  blaster_states = np.zeros((17))
  yref = np.array([0.0, 0.0, 2.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00, 0.0, 0, 0, 0, 0, 0.0, 0, 0])
  blaster_params = {"yref": yref, "mass": mass,"J": J, "l_x": l_x, "l_y": l_y, "N": 200, "yaw_coefficient": yaw_coefficient, "blastThruster": thrusterCoefficient}
  try: 
    blasterController(blaster_params)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

