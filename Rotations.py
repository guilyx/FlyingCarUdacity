#!/usr/bin/env python
# coding: utf-8

# ## Euler Rotations

# Euler rotations as defined in this program are counterclockwise about the axes of the vehicle body frame, where:
#
# - *Roll* - $\phi$ is about the x-axis
# - *Pitch* - $\theta$ is about the y-axis
# - *Yaw* - $\psi$ is about the z-axis
#
# The same set of rotation transformations, applied in a different order can produce a very different final result!


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import math
from enum import Enum

np.set_printoptions(precision=3, suppress=True)
plt.rcParams["figure.figsize"] = [12, 12]


class Rotation(Enum):
    ROLL = 0
    PITCH = 1
    YAW = 2


class EulerRotation:

    def __init__(self, rotations):
        """
        `rotations` is a list of 2-element tuples where the
        first element is the rotation kind and the second element
        is angle in degrees.

        Ex:

            [(Rotation.ROLL, 45), (Rotation.YAW, 32), (Rotation.PITCH, 55)]

        """
        self._rotations = rotations
        self._rotation_map = {Rotation.ROLL: self.roll,
                              Rotation.PITCH: self.pitch, Rotation.YAW: self.yaw}

    def roll(self, phi):
        """Returns a rotation matrix along the roll axis"""
        rollMatrix = np.array([
            [1, 0, 0],
            [0, math.cos(phi), -math.sin(phi)],
            [0, math.sin(phi), math.cos(phi)]
        ])
        return rollMatrix

    def pitch(self, theta):
        """Returns the rotation matrix along the pitch axis"""
        pitchMatrix = np.array([
            [math.cos(theta), 0, math.sin(theta)],
            [0, 1, 0],
            [-math.sin(theta), 0, math.cos(theta)]
        ])
        return pitchMatrix

    def yaw(self, psi):
        """Returns the rotation matrix along the yaw axis"""
        yawMatrix = np.array([
            [math.cos(psi), -math.sin(psi), 0],
            [math.sin(psi), math.cos(psi), 0],
            [0, 0, 1]
        ])
        return yawMatrix

    def rotate(self):
        """Applies the rotations in sequential order"""
        t = np.eye(3)
        for elem in self._rotations:
            rType = elem[0]
            rAngleD = elem[1]
            rAngle = np.pi*rAngleD/180
            matrix = self._rotation_map[rType](rAngle)
            t = np.dot(matrix, t)
        return t


rotations = [
    (Rotation.ROLL, 25),
    (Rotation.PITCH, 75),
    (Rotation.YAW, 90),
]

R = EulerRotation(rotations).rotate()
print('Rotation matrix ...')
print(R)
# Should print
# Rotation matrix ...
# [[ 0.    -0.906  0.423]
#  [ 0.259  0.408  0.875]
#  [-0.966  0.109  0.235]]


# ### Same Rotations, Different Order

rotations = [
    (Rotation.ROLL, 25),
    (Rotation.PITCH, 75),
    (Rotation.YAW, 90),
]

rotations1 = [
    (Rotation.ROLL, 90),
    (Rotation.PITCH, 90),
    (Rotation.YAW, 90),
]

rotations2 = [
    (Rotation.ROLL, 0),
    (Rotation.PITCH, 60),
    (Rotation.YAW, 180),
]

R1 = EulerRotation(rotations).rotate()
R2 = EulerRotation(rotations1).rotate()
R3 = EulerRotation(rotations2).rotate()

print(R1)
print(R2)
print(R3)


# unit vector along x-axis
v = np.array([1, 0, 0])

rv1 = np.dot(R1, v)
rv2 = np.dot(R2, v)
rv3 = np.dot(R3, v)

print(rv1)
print(rv2)
print(rv3)


# Plotting time ...

fig = plt.figure()
ax = fig.gca(projection='3d')

# axes (shown in black)
ax.quiver(0, 0, 0, 1.5, 0, 0, color='black', arrow_length_ratio=0.15)
ax.quiver(0, 0, 0, 0, 1.5, 0, color='black', arrow_length_ratio=0.15)
ax.quiver(0, 0, 0, 0, 0, 1.5, color='black', arrow_length_ratio=0.15)


# Original Vector (shown in blue)
ax.quiver(0, 0, 0, v[0], v[1], v[2], color='blue', arrow_length_ratio=0.15)

# Rotated Vectors (shown in red)
ax.quiver(0, 0, 0, rv1[0], rv1[1], rv1[2],
          color='red', arrow_length_ratio=0.15)
ax.quiver(0, 0, 0, rv2[0], rv2[1], rv2[2],
          color='purple', arrow_length_ratio=0.15)
ax.quiver(0, 0, 0, rv3[0], rv3[1], rv3[2],
          color='green', arrow_length_ratio=0.15)

ax.set_xlim3d(-1, 1)
ax.set_ylim3d(1, -1)
ax.set_zlim3d(1, -1)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()


# ### Gimbal Lock
# Perform rotations with a pitch of +-90 degrees to enter Gimbal Lock.

