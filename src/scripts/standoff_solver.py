#!/usr/bin/env python

import numpy as np

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class standoffSolver:
  def __init__(self, pitch, roll, standoff, blasterX, blasterY, blasterZ):
    self.pitch = np.deg2rad(pitch)
    self.roll = np.deg2rad(roll)
    self.blaster_X = blasterX
    self.blaster_Y = blasterY
    self.blaster_Z = blasterZ
    self.hypo = standoff
    self.adj = 0.0
    self.opp = 0.0
    pass

  def solve(self):
    self.opp = self.hypo*np.sin(self.pitch)
    self.adj = self.hypo*np.cos(self.pitch)
    return self.opp, self.adj


if __name__ == '__main__':
  run = standoffSolver(10, 0, 1.5, 1.0, 0.0, 2.0)
  opp, adj = run.solve()
  print(round(opp, 4))
  print(round(adj, 4))
  fig = plt.figure()
  ax = plt.axes(projection='3d')
  ax.scatter3D(1.0, 0.0, 2.0)
  ax.scatter3D(1.0+opp, 0.0, 2.0-adj)
  plt.show()

