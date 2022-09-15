import numpy as np

from casadi import *


class sensing:
  def __init__(self, alph1, alph2, dne):
    self.alpha1 = np.deg2rad(alph1)
    self.alpha2 = np.deg2rad(alph2)
    self.dne = dne

  def rot_gim(self, alpha1, alpha2):
    R_gimbal_1 = SX.eye(3)
    R_gimbal_2 = SX.eye(3)

    # Rotation about x. 

    R_gimbal_2[1, 1] = cos(self.alpha2)
    R_gimbal_2[1, 2] = -sin(self.alpha2)
    R_gimbal_2[2, 1] = sin(self.alpha2)
    R_gimbal_2[2, 2] = cos(self.alpha2)

    # Rotation about y.

    R_gimbal_1[0, 0] = cos(self.alpha1)
    R_gimbal_1[0, 2] = sin(self.alpha1)
    R_gimbal_1[2, 0] = -sin(self.alpha1)
    R_gimbal_1[2, 2] = cos(self.alpha1)

    R_gimbal = R_gimbal_1 @ R_gimbal_2 # body to nozzle rotation.
    return R_gimbal

  def rot_e(self):
    pass